#!/usr/bin/env python3
"""Validate frozen UAV reviewer runs and recompute network metrics from raw traces.

The validator intentionally treats process completion, ONNX replay, mobility,
physical channel inputs, satellite measurements, and traffic denominators as
separate gates.  It never infers a denominator from a screenshot or aggregate.
The v10 protocol fixes initial access association; handover success and UE
ping-pong are therefore reported as NA and any handover event fails the run.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
import re
import struct
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Iterable


MODEL_SHA256 = "2f9bb76fe6d161cf8d4ab6ff5031647f45fa7cb6916d60463b4cffadeca1207f"
SCHEMA_SHA256 = "7602c73891c528d2972cbf078032d9f42c4ef6de591f45bdcb2b3ccb9af2dbb9"
FEATURE_COLUMNS = (
    "BestDonorDistanceM",
    "XhaulConnected",
    "XhaulChannelVariationDb",
    "XhaulRsrpDbm",
    "RawXhaulStateAgeSec",
    "DeltaDistance2sM",
    "DeltaRsrp2sDb",
    "DeltaRsrp4sDb",
)
EXPECTED_UAV_CELL_IDS = (5, 6, 7)
PHYSICAL_XHAUL_COLUMNS = (
    "Time",
    "UavIndex",
    "UavCellId",
    "BestDonorCellId",
    "BestDonorDistanceM",
    "XhaulConnected",
    "XhaulChannelVariationDb",
    "XhaulRsrpDbm",
    "RawXhaulState",
    "RawXhaulStateAgeSec",
    "XhaulDegradationActive",
    "XhaulDegradationPenaltyDb",
    "DonorUnavailableActive",
)
SATELLITE_COLUMNS = (
    "Time",
    "CellId",
    "ServiceDlSnrDb",
    "ServiceUlSnrDb",
    "FeederDlSnrDb",
    "FeederUlSnrDb",
    "BackhaulDlSnrDb",
    "BackhaulUlSnrDb",
    "BackhaulHealthy",
)
QOS_TX_COLUMNS = ("Time", "UE", "Dir", "IntervalSec", "TxPackets")
PACKET_OFFER_COLUMNS = ("Time", "UE", "Dir", "PacketBytes", "Sequence")
PROVENANCE_PATH_HASH_FIELDS = (
    ("helper_path", "helper_sha256"),
    ("nr_library_path", "nr_library_sha256"),
    ("runner_path", "runner_sha256"),
    ("validator_path", "validator_sha256"),
    ("campaign_runner_path", "campaign_runner_sha256"),
    ("route_contract_report_path", "route_contract_report_sha256"),
    ("route_contract_manifest_path", "route_contract_manifest_sha256"),
    ("cmake_cache_path", "cmake_cache_sha256"),
    ("matched_config_path", "matched_config_sha256"),
    ("policy_config_path", "policy_config_sha256"),
    ("full_config_path", "full_config_sha256"),
    ("git_status_path", "git_status_sha256"),
    ("git_diff_path", "git_diff_sha256"),
    ("build_provenance_path", "build_provenance_sha256"),
    ("cmake_cache_archive_path", "cmake_cache_archive_sha256"),
    ("frozen_inputs_manifest_path", "frozen_inputs_manifest_sha256"),
)
MATCHED_PROVENANCE_HASH_FIELDS = (
    "helper_sha256",
    "nr_library_sha256",
    "runner_sha256",
    "validator_sha256",
    "campaign_runner_sha256",
    "route_contract_report_sha256",
    "route_contract_manifest_sha256",
    "cmake_cache_sha256",
    "git_diff_sha256",
    "cmake_cache_archive_sha256",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        value = json.load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object in {path}")
    return value


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def read_argv0(path: Path) -> list[str]:
    """Read the runner's NUL-delimited argument manifest without shell parsing."""
    payload = path.read_bytes()
    if payload and not payload.endswith(b"\0"):
        raise ValueError(f"argument manifest is not NUL terminated: {path}")
    return [item.decode("utf-8") for item in payload.split(b"\0") if item]


def option_map(arguments: list[str]) -> tuple[dict[str, str], list[str]]:
    """Return --name=value options and report duplicate option names."""
    options: dict[str, str] = {}
    duplicates: list[str] = []
    for argument in arguments:
        if not argument.startswith("--") or "=" not in argument:
            continue
        name, value = argument.split("=", 1)
        if name in options:
            duplicates.append(name)
        options[name] = value
    return options, duplicates


def number(value: str | None, default: float = math.nan) -> float:
    try:
        return float(value) if value not in (None, "") else default
    except (TypeError, ValueError):
        return default


def integer(value: str | None, default: int = 0) -> int:
    value_as_number = number(value)
    return int(value_as_number) if math.isfinite(value_as_number) else default


def flag(value: str | None) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def rate(numerator: float, denominator: float) -> float | None:
    return numerator / denominator if denominator else None


def gate(name: str, passed: bool, **details: Any) -> dict[str, Any]:
    return {"name": name, "passed": bool(passed), "details": details}


def max_csv_time(path: Path) -> float | None:
    maximum: float | None = None
    for row in read_csv_rows(path):
        value = number(row.get("Time"))
        if math.isfinite(value) and (maximum is None or value > maximum):
            maximum = value
    return maximum


def max_text_time(path: Path) -> float | None:
    maximum: float | None = None
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            token = line.split(maxsplit=1)[0] if line.strip() else ""
            value = number(token)
            if math.isfinite(value) and (maximum is None or value > maximum):
                maximum = value
    return maximum


def parse_handover_line(line: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    parts = line.strip().split()
    if parts:
        fields["Time"] = parts[0]
    for key, value in re.findall(r"([A-Za-z][A-Za-z0-9_]*)=([^\s]+)", line):
        fields[key] = value
    return fields


def handover_metrics(trace: Path, failure_trace: Path, pingpong_window_s: float) -> dict[str, Any]:
    events: list[dict[str, str]] = []
    if trace.exists():
        with trace.open(encoding="utf-8", errors="replace") as stream:
            events = [parse_handover_line(line) for line in stream if line.strip()]

    starts = [event for event in events if event.get("Event") == "START"]
    endings = [event for event in events if event.get("Event") == "END_OK"]
    failure_events: list[dict[str, str]] = []
    if failure_trace.exists():
        with failure_trace.open(encoding="utf-8", errors="replace") as stream:
            failure_events = [parse_handover_line(line) for line in stream if line.strip()]

    def attempt_id(event: dict[str, str]) -> str | None:
        value = event.get("AttemptId", "")
        return value if value not in {"", "0"} else None

    def legacy_key(event: dict[str, str], outcome: bool = False) -> tuple[str, str, str, str]:
        return (
            event.get("IMSI", ""),
            event.get("SourceCell", ""),
            event.get("TargetCell", ""),
            event.get("StartTime", "") if outcome else event.get("Time", ""),
        )

    starts_by_attempt: dict[str, dict[str, str]] = {}
    starts_by_legacy: dict[tuple[str, str, str, str], dict[str, str]] = {}
    duplicate_start_attempt_ids: list[str] = []
    duplicate_legacy_start_keys: list[tuple[str, str, str, str]] = []
    zero_start_attempt_ids = 0
    for start in starts:
        if start.get("AttemptId") == "0":
            zero_start_attempt_ids += 1
        identifier = attempt_id(start)
        if identifier is not None:
            if identifier in starts_by_attempt:
                duplicate_start_attempt_ids.append(identifier)
            else:
                starts_by_attempt[identifier] = start
        key = legacy_key(start)
        if key in starts_by_legacy:
            duplicate_legacy_start_keys.append(key)
        else:
            starts_by_legacy[key] = start

    matched_start_ids: set[int] = set()
    matched_endings: list[dict[str, str]] = []
    matched_failures: list[dict[str, str]] = []
    unmatched_outcomes: list[dict[str, Any]] = []
    mismatched_outcomes: list[dict[str, Any]] = []
    duplicate_outcome_attempt_ids: list[str] = []
    seen_outcome_attempts: set[str] = set()
    unlinked_failure_callback_records: list[dict[str, str]] = []

    def match_outcome(event: dict[str, str], outcome_name: str) -> dict[str, str] | None:
        if event.get("AttemptId") == "0" and outcome_name == "FAILURE":
            # A second UE/GNB failure callback can legitimately arrive after
            # the first callback has consumed and cleared the active attempt.
            # It is a diagnostic record, not another failed attempt.
            unlinked_failure_callback_records.append(event)
            return None
        identifier = attempt_id(event)
        if identifier is not None:
            if identifier in seen_outcome_attempts:
                duplicate_outcome_attempt_ids.append(identifier)
                return None
            seen_outcome_attempts.add(identifier)
            start = starts_by_attempt.get(identifier)
        else:
            start = starts_by_legacy.get(legacy_key(event, outcome=True))
        if start is None:
            unmatched_outcomes.append(
                {"outcome": outcome_name, "attempt_id": identifier, "event": event}
            )
            return None
        expected = legacy_key(start)
        observed = legacy_key(event, outcome=True)
        if expected != observed:
            mismatched_outcomes.append(
                {
                    "outcome": outcome_name,
                    "attempt_id": identifier,
                    "expected": expected,
                    "observed": observed,
                }
            )
            return None
        object_id = id(start)
        if object_id in matched_start_ids:
            mismatched_outcomes.append(
                {"outcome": outcome_name, "attempt_id": identifier, "reason": "start reused"}
            )
            return None
        matched_start_ids.add(object_id)
        return start

    for ending in endings:
        if match_outcome(ending, "END_OK") is not None:
            matched_endings.append(ending)
    for failure in failure_events:
        if match_outcome(failure, "FAILURE") is not None:
            matched_failures.append(failure)

    by_imsi: dict[str, list[dict[str, str]]] = defaultdict(list)
    for event in matched_endings:
        by_imsi[event.get("IMSI", "")].append(event)
    pingpong_events = 0
    eligible_pairs = 0
    affected_ues: set[str] = set()
    for imsi, sequence in by_imsi.items():
        sequence.sort(key=lambda item: number(item.get("Time"), -1.0))
        for previous, current in zip(sequence, sequence[1:]):
            elapsed = number(current.get("Time")) - number(previous.get("Time"))
            if 0.0 <= elapsed <= pingpong_window_s:
                eligible_pairs += 1
                reversed_pair = (
                    previous.get("SourceCell") == current.get("TargetCell")
                    and previous.get("TargetCell") == current.get("SourceCell")
                )
                if reversed_pair:
                    pingpong_events += 1
                    affected_ues.add(imsi)

    completion_delays = [
        number(event.get("CompletionDelay"))
        for event in matched_endings
        if math.isfinite(number(event.get("CompletionDelay")))
        and number(event.get("CompletionDelay")) >= 0.0
    ]
    malformed_completion_delays = sum(
        not math.isfinite(number(event.get("CompletionDelay")))
        or number(event.get("CompletionDelay")) < 0.0
        for event in matched_endings
    )
    unresolved_starts = len(starts) - len(matched_endings) - len(matched_failures)
    trace_integrity_passed = not any(
        (
            duplicate_start_attempt_ids,
            duplicate_legacy_start_keys,
            duplicate_outcome_attempt_ids,
            unmatched_outcomes,
            mismatched_outcomes,
        )
    ) and malformed_completion_delays == 0 and zero_start_attempt_ids == 0 and unresolved_starts == 0
    return {
        "start_events": len(starts),
        "end_ok_events": len(endings),
        "matched_start_end_ok": len(matched_endings),
        "matched_start_failure": len(matched_failures),
        "unresolved_starts": unresolved_starts,
        "unmatched_starts": len(starts) - len(matched_endings),
        "starts_without_success": len(starts) - len(matched_endings),
        "unmatched_end_ok": len(endings) - len(matched_endings),
        "failure_trace_records": len(failure_events),
        "success_rate": rate(len(matched_endings), len(starts)),
        "success_denominator_start_events": len(starts),
        "mean_completion_delay_s": (
            sum(completion_delays) / len(completion_delays) if completion_delays else None
        ),
        "pingpong_window_s": pingpong_window_s,
        "pingpong_events": pingpong_events,
        "pingpong_eligible_adjacent_pairs": eligible_pairs,
        "pingpong_rate": rate(pingpong_events, eligible_pairs),
        "pingpong_affected_ues": len(affected_ues),
        "trace_integrity": {
            "passed": trace_integrity_passed,
            "matching_rule": "AttemptId when nonzero; otherwise exact IMSI/source/target/StartTime",
            "duplicate_start_attempt_ids": duplicate_start_attempt_ids[:20],
            "zero_start_attempt_ids": zero_start_attempt_ids,
            "duplicate_legacy_start_keys": duplicate_legacy_start_keys[:20],
            "duplicate_outcome_attempt_ids": duplicate_outcome_attempt_ids[:20],
            "unmatched_outcome_count": len(unmatched_outcomes),
            "first_unmatched_outcomes": unmatched_outcomes[:20],
            "mismatched_outcome_count": len(mismatched_outcomes),
            "first_mismatched_outcomes": mismatched_outcomes[:20],
            "malformed_completion_delays": malformed_completion_delays,
            "unresolved_starts": unresolved_starts,
            "unlinked_failure_callback_records": len(unlinked_failure_callback_records),
            "first_unlinked_failure_callbacks": unlinked_failure_callback_records[:20],
        },
    }


def qos_bucket(rows: Iterable[dict[str, str]]) -> dict[str, Any]:
    tx = rx = 0
    rx_bytes = 0
    delay_sum = jitter_sum = 0.0
    for row in rows:
        tx += integer(row.get("TxPackets"))
        rx += integer(row.get("RxPackets"))
        rx_bytes += integer(row.get("RxBytes"))
        delay_sum += number(row.get("DelaySumSec"), 0.0)
        jitter_sum += number(row.get("JitterSumSec"), 0.0)
    return {
        "tx_packets": tx,
        "rx_packets": rx,
        "lost_packets": tx - rx,
        "rx_bytes": rx_bytes,
        "pdr": rate(rx, tx),
        "pdr_denominator_tx_packets": tx,
        "mean_delay_s": rate(delay_sum, rx),
        "mean_jitter_s": rate(jitter_sum, max(0, rx - 1)),
    }


def qos_metrics(path: Path, degradation_start: float, degradation_stop: float) -> dict[str, Any]:
    rows = read_csv_rows(path)
    windows = {
        "full": (10.0, math.inf, True),
        "pre_degradation": (10.0, degradation_start, False),
        "degradation": (degradation_start, degradation_stop, False),
        "post_degradation": (degradation_stop, math.inf, True),
    }
    result: dict[str, Any] = {
        "scope": "monitored UES1 UDP flows only",
        "window_rule": "component intervals [start,stop); final stop inclusive",
        "windows": {},
    }
    for window_name, (start, stop, include_stop) in windows.items():
        selected = [
            row
            for row in rows
            if number(row.get("Time")) >= start
            and (number(row.get("Time")) <= stop if include_stop else number(row.get("Time")) < stop)
        ]
        by_direction = {
            direction: qos_bucket(row for row in selected if row.get("Dir") == direction)
            for direction in ("DL", "UL")
        }
        by_direction["ALL"] = qos_bucket(selected)
        result["windows"][window_name] = by_direction
    return result


def packet_cohort_metrics(
    path: Path,
    monitored_ues: int,
    sim_time: float,
    degradation_start: float,
    degradation_stop: float,
    packet_size_bytes: int = 1000,
) -> dict[str, Any]:
    """Compute offered-packet cohorts and the Eq. (14)/(15) UE indicator.

    Application TX attempts are the denominator. A received packet is assigned
    to the window/bin containing its sender timestamp, even if propagation
    carries the RX event over a window boundary.
    """
    rows = read_csv_rows(path)

    def packet_key(row: dict[str, str]) -> tuple[int, str, int] | None:
        ue_value = number(row.get("UE"))
        sequence_value = number(row.get("Sequence"))
        direction = row.get("Dir", "")
        if (
            not math.isfinite(ue_value)
            or ue_value < 0.0
            or not ue_value.is_integer()
            or direction not in {"DL", "UL"}
            or not math.isfinite(sequence_value)
            or sequence_value < 0.0
            or not sequence_value.is_integer()
        ):
            return None
        return (int(ue_value), direction, int(sequence_value))

    tx_raw = [row for row in rows if row.get("Event") == "TX"]
    rx_raw = [row for row in rows if row.get("Event") == "RX"]
    unknown_event_records = sum(row.get("Event") not in {"TX", "RX"} for row in rows)
    tx_keys = [packet_key(row) for row in tx_raw]
    rx_keys = [packet_key(row) for row in rx_raw]
    valid_tx_keys = [key for key in tx_keys if key is not None]
    valid_rx_keys = [key for key in rx_keys if key is not None]
    tx_counts = Counter(valid_tx_keys)
    rx_counts = Counter(valid_rx_keys)
    tx_by_key: dict[tuple[int, str, int], dict[str, str]] = {}
    rx_by_key: dict[tuple[int, str, int], dict[str, str]] = {}
    for row, key in zip(tx_raw, tx_keys):
        if key is not None and key not in tx_by_key:
            tx_by_key[key] = row
    for row, key in zip(rx_raw, rx_keys):
        if key is not None and key not in rx_by_key:
            rx_by_key[key] = row

    invalid_time_records = 0
    invalid_packet_size_records = 0
    invalid_delay_records = 0
    mismatched_records: list[dict[str, Any]] = []
    for row in rows:
        timestamp = number(row.get("Time"))
        packet_bytes = number(row.get("PacketBytes"))
        if not math.isfinite(timestamp) or timestamp < 0.0:
            invalid_time_records += 1
        if not math.isfinite(packet_bytes) or packet_bytes <= 0.0:
            invalid_packet_size_records += 1

    for row, key in zip(tx_raw, tx_keys):
        sender_time = number(row.get("SenderTimestampSec"))
        tx_time = number(row.get("Time"))
        if (
            not math.isfinite(sender_time)
            or sender_time < 0.0
            or not math.isfinite(tx_time)
            or abs(sender_time - tx_time) > 1e-6
        ):
            mismatched_records.append(
                {"event": "TX", "key": key, "time": tx_time, "sender_time": sender_time}
            )

    for row, key in zip(rx_raw, rx_keys):
        receive_time = number(row.get("Time"))
        sender_time = number(row.get("SenderTimestampSec"))
        if (
            not math.isfinite(receive_time)
            or not math.isfinite(sender_time)
            or sender_time < 0.0
            or receive_time < sender_time
        ):
            invalid_delay_records += 1
            continue
        tx = tx_by_key.get(key) if key is not None else None
        if tx is not None and abs(number(tx.get("Time")) - sender_time) > 1e-6:
            mismatched_records.append(
                {
                    "event": "RX",
                    "key": key,
                    "tx_time": number(tx.get("Time")),
                    "sender_time": sender_time,
                }
            )

    duplicate_tx_records = sum(count - 1 for count in tx_counts.values())
    duplicate_rx_records = sum(count - 1 for count in rx_counts.values())
    orphan_rx_keys = [key for key in rx_by_key if key not in tx_by_key]
    packet_integrity = {
        "passed": False,
        "pairing_key": ["UE", "Dir", "Sequence"],
        "raw_tx_records": len(tx_raw),
        "raw_rx_records": len(rx_raw),
        "unique_valid_tx_keys": len(tx_by_key),
        "unique_valid_rx_keys": len(rx_by_key),
        "missing_or_invalid_tx_key_records": sum(key is None for key in tx_keys),
        "missing_or_invalid_rx_key_records": sum(key is None for key in rx_keys),
        "duplicate_tx_records": duplicate_tx_records,
        "duplicate_rx_records": duplicate_rx_records,
        "orphan_rx_records": len(orphan_rx_keys),
        "first_orphan_rx_keys": orphan_rx_keys[:20],
        "mismatched_record_count": len(mismatched_records),
        "first_mismatched_records": mismatched_records[:20],
        "invalid_time_records": invalid_time_records,
        "invalid_packet_size_records": invalid_packet_size_records,
        "invalid_delay_records": invalid_delay_records,
        "unknown_event_records": unknown_event_records,
        "undelivered_tx_records": sum(key not in rx_by_key for key in tx_by_key),
    }
    packet_integrity["passed"] = all(
        packet_integrity[name] == 0
        for name in (
            "missing_or_invalid_tx_key_records",
            "missing_or_invalid_rx_key_records",
            "duplicate_tx_records",
            "duplicate_rx_records",
            "orphan_rx_records",
            "mismatched_record_count",
            "invalid_time_records",
            "invalid_packet_size_records",
            "invalid_delay_records",
            "unknown_event_records",
        )
    )
    tx_rows = list(tx_by_key.values())
    thresholds = {
        "DL": {"min_rate_mbps": 0.18, "max_delay_s": 0.30, "min_pdr": 0.90},
        "UL": {"min_rate_mbps": 0.045, "max_delay_s": 0.30, "min_pdr": 0.90},
    }
    windows = {
        "full": (10.0, sim_time, True),
        "pre_degradation": (10.0, degradation_start, False),
        "degradation": (degradation_start, degradation_stop, False),
        "post_degradation": (degradation_stop, sim_time, True),
    }

    def in_window(timestamp: float, start: float, stop: float, include_stop: bool) -> bool:
        return timestamp >= start and (timestamp <= stop if include_stop else timestamp < stop)

    def summarize(selected: list[dict[str, str]]) -> dict[str, Any]:
        delivered: list[dict[str, str]] = []
        delays: list[float] = []
        for row in selected:
            key = packet_key(row)
            rx = rx_by_key.get(key) if key is not None else None
            if rx is not None:
                delivered.append(rx)
                sender_time = number(rx.get("SenderTimestampSec"))
                receive_time = number(rx.get("Time"))
                if (
                    math.isfinite(sender_time)
                    and math.isfinite(receive_time)
                    and sender_time >= 0.0
                    and receive_time >= sender_time
                ):
                    delays.append(receive_time - sender_time)
        return {
            "tx_attempts": len(selected),
            "rx_packets": len(delivered),
            "lost_packets": len(selected) - len(delivered),
            "pdr": rate(len(delivered), len(selected)),
            "pdr_denominator_tx_attempts": len(selected),
            "mean_delay_s": sum(delays) / len(delays) if delays else None,
            "max_delay_s": max(delays) if delays else None,
        }

    window_metrics: dict[str, Any] = {}
    for name, (start, stop, include_stop) in windows.items():
        selected = [
            row
            for row in tx_rows
            if in_window(number(row.get("Time")), start, stop, include_stop)
        ]
        by_direction = {
            direction: summarize([row for row in selected if row.get("Dir") == direction])
            for direction in ("DL", "UL")
        }
        by_direction["ALL"] = summarize(selected)
        window_metrics[name] = by_direction

    # Build one global <=2 s grid and insert every analysis boundary. Thus the
    # pre/degradation/post service samples exactly partition the full window;
    # a boundary cutting a regular bin creates two shorter bins, not a gap or
    # a double-counted sample.
    bin_seconds = 2.0
    boundaries = {10.0, degradation_start, degradation_stop, sim_time}
    next_boundary = 10.0
    while next_boundary < sim_time:
        boundaries.add(next_boundary)
        next_boundary += bin_seconds
    boundaries = {value for value in boundaries if 10.0 <= value <= sim_time}
    ordered_boundaries = sorted(boundaries)
    bins = list(zip(ordered_boundaries, ordered_boundaries[1:]))
    per_bin: dict[tuple[int, int, str], list[dict[str, str]]] = defaultdict(list)
    for row in tx_rows:
        timestamp = number(row.get("Time"))
        if timestamp < 10.0 or timestamp > sim_time or not bins:
            continue
        bin_index = min(bisect.bisect_right(ordered_boundaries, timestamp) - 1, len(bins) - 1)
        per_bin[(bin_index, integer(row.get("UE")), row.get("Dir", ""))].append(row)

    bin_indicators: list[tuple[float, float, list[int]]] = []
    for bin_index, (bin_start, bin_stop) in enumerate(bins):
        bin_duration = bin_stop - bin_start
        indicators: list[int] = []
        for ue in range(monitored_ues):
            passed = True
            for direction in ("DL", "UL"):
                cohort = per_bin.get((bin_index, ue, direction), [])
                summary = summarize(cohort)
                delivered = summary["rx_packets"]
                observed_rate = delivered * packet_size_bytes * 8.0 / bin_duration / 1e6
                criterion = thresholds[direction]
                passed &= (
                    summary["tx_attempts"] > 0
                    and summary["pdr"] is not None
                    and summary["mean_delay_s"] is not None
                    and observed_rate >= criterion["min_rate_mbps"]
                    and summary["mean_delay_s"] <= criterion["max_delay_s"]
                    and summary["pdr"] >= criterion["min_pdr"]
                )
            indicators.append(int(passed))
        bin_indicators.append((bin_start, bin_stop, indicators))

    service_windows: dict[str, Any] = {}
    for name, (start, stop, _include_stop) in windows.items():
        selected_bins = [
            (bin_start, bin_stop, indicators)
            for bin_start, bin_stop, indicators in bin_indicators
            if bin_start >= start and bin_stop <= stop
        ]
        indicators = [value for _, _, values in selected_bins for value in values]
        service_windows[name] = {
            "passing_ue_samples": sum(indicators),
            "denominator_ue_samples": len(indicators),
            "rate": rate(sum(indicators), len(indicators)),
            "time_samples": len(selected_bins),
            "bin_durations_s": [stop - start for start, stop, _ in selected_bins],
        }

    packet_partition = {
        direction: window_metrics["full"][direction]["tx_attempts"]
        == sum(
            window_metrics[name][direction]["tx_attempts"]
            for name in ("pre_degradation", "degradation", "post_degradation")
        )
        for direction in ("DL", "UL", "ALL")
    }
    service_partition = service_windows["full"]["denominator_ue_samples"] == sum(
        service_windows[name]["denominator_ue_samples"]
        for name in ("pre_degradation", "degradation", "post_degradation")
    )

    return {
        "cohort_rule": (
            "pair by (UE,Dir,Sequence); application TX timestamp; disjoint intervals "
            "[start,stop), final simulation stop inclusive"
        ),
        "tx_attempt_records": len(tx_raw),
        "unique_valid_tx_attempts": len(tx_rows),
        "rx_records": len(rx_raw),
        "packet_integrity": packet_integrity,
        "window_partition": {
            "passed": all(packet_partition.values()) and service_partition,
            "packet_tx_partition_by_direction": packet_partition,
            "service_denominator_partition": service_partition,
        },
        "windows": window_metrics,
        "service_satisfaction": {
            "method": (
                "global <=2 s per-UE packet cohorts split at analysis boundaries; "
                "both DL and UL criteria must pass"
            ),
            "monitored_ues_denominator": monitored_ues,
            "thresholds": thresholds,
            "windows": service_windows,
        },
    }


def switching_metrics(
    event_path: Path,
    xhaul_path: Path,
    degradation_start: float,
    degradation_stop: float,
    policy: str,
) -> dict[str, Any]:
    events = read_csv_rows(event_path)
    xhaul = read_csv_rows(xhaul_path)
    policy_events = [
        row for row in events if row.get("EventType", "POLICY_APPLY") == "POLICY_APPLY"
    ]
    changed = [row for row in policy_events if flag(row.get("ModeChanged"))]
    route_changed = [row for row in events if flag(row.get("RouteChanged"))]
    cells = sorted({integer(row.get("UavCellId")) for row in xhaul})
    per_cell: dict[str, Any] = {}
    for cell in cells:
        cell_rows = sorted(
            (row for row in xhaul if integer(row.get("UavCellId")) == cell),
            key=lambda row: number(row.get("Time")),
        )
        cell_events = sorted(
            (row for row in changed if integer(row.get("UavCellId")) == cell),
            key=lambda row: number(row.get("ApplyTime")),
        )
        cell_route_events = sorted(
            (row for row in route_changed if integer(row.get("UavCellId")) == cell),
            key=lambda row: number(row.get("ApplyTime")),
        )
        raw_episodes: list[dict[str, Any]] = []
        current_episode: dict[str, Any] | None = None
        previous_time: float | None = None
        previous_unreachable = False
        for row in cell_rows:
            timestamp = number(row.get("Time"))
            unreachable = row.get("RawXhaulState") == "UNREACHABLE"
            contiguous = (
                previous_time is not None
                and previous_unreachable
                and abs(timestamp - previous_time - 2.0) <= 1e-6
            )
            if unreachable:
                if current_episode is None or not contiguous:
                    if current_episode is not None:
                        raw_episodes.append(current_episode)
                    current_episode = {
                        "onset_s": timestamp,
                        "last_unreachable_s": timestamp,
                        "first_non_unreachable_s": None,
                        "unreachable_samples": 1,
                    }
                else:
                    current_episode["last_unreachable_s"] = timestamp
                    current_episode["unreachable_samples"] += 1
            elif current_episode is not None:
                current_episode["first_non_unreachable_s"] = timestamp
                raw_episodes.append(current_episode)
                current_episode = None
            previous_time = timestamp
            previous_unreachable = unreachable
        if current_episode is not None:
            raw_episodes.append(current_episode)
        raw_episodes = [
            episode
            for episode in raw_episodes
            if episode["onset_s"] <= degradation_stop
            and episode["last_unreachable_s"] >= degradation_start
        ]

        satellite_policy_events = [
            row
            for row in cell_events
            if row.get("AppliedMode") == "SATELLITE_FALLBACK"
            and row.get("ActualRouteMode") == "SATELLITE_FALLBACK"
        ]
        xhaul_by_time = {
            round(number(row.get("Time")), 9): row for row in cell_rows
        }
        linked_satellite_events: list[dict[str, Any]] = []
        unlinked_satellite_events: list[dict[str, Any]] = []
        for event in satellite_policy_events:
            decision_time = number(event.get("DecisionTime"))
            apply_time = number(event.get("ApplyTime"))
            linked_episode: dict[str, Any] | None = None
            link_rule = ""
            future_raw_targets: list[dict[str, Any]] = []
            if policy == "predictor":
                target_times = [decision_time + offset for offset in (2.0, 4.0, 6.0)]
                target_rows = [xhaul_by_time.get(round(timestamp, 9)) for timestamp in target_times]
                future_raw_targets = [
                    {
                        "offset_s": offset,
                        "time_s": timestamp,
                        "raw_xhaul_state": (
                            row.get("RawXhaulState") if row is not None else None
                        ),
                    }
                    for offset, timestamp, row in zip(
                        (2.0, 4.0, 6.0), target_times, target_rows
                    )
                ]
                if all(
                    row is not None and row.get("RawXhaulState") == "UNREACHABLE"
                    for row in target_rows
                ):
                    candidates = [
                        episode
                        for episode in raw_episodes
                        if episode["onset_s"] <= target_times[0]
                        and episode["last_unreachable_s"] >= target_times[-1]
                    ]
                    if candidates:
                        linked_episode = max(
                            candidates, key=lambda episode: episode["onset_s"]
                        )
                        link_rule = "exact UNREACHABLE target at decision +2/+4/+6 s"
            else:
                required_raw_age_s = 0.0 if policy == "fast-rule" else 5.0
                candidates = [
                    episode
                    for episode in raw_episodes
                    if episode["onset_s"] <= decision_time
                    and decision_time - episode["onset_s"] + 1e-6
                    >= required_raw_age_s
                    and (
                        episode["first_non_unreachable_s"] is None
                        or decision_time < episode["first_non_unreachable_s"]
                        or apply_time <= episode["first_non_unreachable_s"] + 0.15 + 1e-6
                    )
                ]
                if candidates:
                    linked_episode = max(candidates, key=lambda episode: episode["onset_s"])
                    link_rule = (
                        "continuous raw episode at decision/apply; required raw age "
                        f"{required_raw_age_s:g} s; 0.15 s actuation allowance"
                    )
            if linked_episode is None:
                unlinked_satellite_events.append(
                    {
                        "uav_cell_id": cell,
                        "decision_time_s": decision_time,
                        "apply_time_s": apply_time,
                        "controller_mode": event.get("ControllerMode"),
                        "future_raw_targets": future_raw_targets,
                    }
                )
            else:
                linked_satellite_events.append(
                    {
                        "uav_cell_id": cell,
                        "decision_time_s": decision_time,
                        "apply_time_s": apply_time,
                        "raw_onset_s": linked_episode["onset_s"],
                        "raw_last_unreachable_s": linked_episode["last_unreachable_s"],
                        "episode_id": f"{cell}:{linked_episode['onset_s']:.9f}",
                        "decision_minus_raw_onset_s": decision_time
                        - linked_episode["onset_s"],
                        "apply_minus_raw_onset_s": apply_time
                        - linked_episode["onset_s"],
                        "link_rule": link_rule,
                        "future_raw_targets": future_raw_targets,
                    }
                )

        route_history = [(-math.inf, "TN_DIRECT")]
        route_history.extend(
            (number(row.get("ApplyTime")), row.get("ActualRouteMode", ""))
            for row in cell_route_events
        )
        route_history.sort(key=lambda entry: entry[0])

        def route_mode_at(timestamp: float) -> str:
            mode = "TN_DIRECT"
            for apply_time, applied_mode in route_history:
                if apply_time > timestamp:
                    break
                mode = applied_mode
            return mode

        def episode_recovery(episode: dict[str, Any]) -> float | None:
            later_onsets = [
                candidate["onset_s"]
                for candidate in raw_episodes
                if candidate["onset_s"] > episode["onset_s"]
            ]
            next_onset = min(later_onsets) if later_onsets else math.inf
            for row_index, row in enumerate(cell_rows[:-1]):
                timestamp = number(row.get("Time"))
                next_row = cell_rows[row_index + 1]
                if (
                    episode["last_unreachable_s"] < timestamp < next_onset
                    and row.get("RawXhaulState") == "HEALTHY"
                    and next_row.get("RawXhaulState") == "HEALTHY"
                    and number(next_row.get("Time")) < next_onset
                    and abs(number(next_row.get("Time")) - timestamp - 2.0) <= 1e-6
                ):
                    return timestamp
            return None

        tn_policy_events = [
            row
            for row in cell_events
            if row.get("AppliedMode") == "TN_DIRECT"
            and row.get("ActualRouteMode") == "TN_DIRECT"
        ]
        for link in linked_satellite_events:
            episode = next(
                episode
                for episode in raw_episodes
                if episode["onset_s"] == link["raw_onset_s"]
            )
            recovery_time = episode_recovery(episode)
            later_onsets = [
                candidate["onset_s"]
                for candidate in raw_episodes
                if candidate["onset_s"] > episode["onset_s"]
            ]
            next_onset = min(later_onsets) if later_onsets else math.inf
            tn_event = next(
                (
                    row
                    for row in tn_policy_events
                    if recovery_time is not None
                    and recovery_time <= number(row.get("DecisionTime")) < next_onset
                ),
                None,
            )
            route_at_onset = route_mode_at(episode["onset_s"])
            route_losses = [
                number(row.get("ApplyTime"))
                for row in cell_route_events
                if episode["onset_s"] <= number(row.get("ApplyTime"))
                <= link["apply_time_s"]
                and row.get("ActualRouteMode") == "NO_BACKHAUL_AVAILABLE"
            ]
            if route_at_onset == "SATELLITE_FALLBACK":
                interruption = 0.0
            elif route_losses:
                interruption = max(0.0, link["apply_time_s"] - min(route_losses))
            else:
                interruption = 0.0
            link.update(
                {
                    "route_mode_at_raw_onset": route_at_onset,
                    "first_route_loss_s": min(route_losses) if route_losses else None,
                    "route_interruption_s": interruption,
                    "anticipatory_zero_route_interruption": (
                        route_at_onset == "SATELLITE_FALLBACK"
                        and link["apply_time_s"] <= episode["onset_s"]
                    ),
                    "first_sustained_raw_healthy_s": recovery_time,
                    "tn_recovery_decision_s": (
                        number(tn_event.get("DecisionTime")) if tn_event is not None else None
                    ),
                    "tn_recovery_apply_s": (
                        number(tn_event.get("ApplyTime")) if tn_event is not None else None
                    ),
                }
            )

        primary_link = linked_satellite_events[0] if linked_satellite_events else None
        primary_episode = None
        if primary_link is not None:
            primary_episode = next(
                (
                    episode
                    for episode in raw_episodes
                    if episode["onset_s"] == primary_link["raw_onset_s"]
                ),
                None,
            )
        elif raw_episodes:
            no_route_times = [
                number(row.get("ApplyTime"))
                for row in cell_route_events
                if row.get("ActualRouteMode") == "NO_BACKHAUL_AVAILABLE"
            ]
            primary_episode = next(
                (
                    episode
                    for episode in raw_episodes
                    if any(
                        episode["onset_s"] <= timestamp
                        and (
                            episode["first_non_unreachable_s"] is None
                            or timestamp <= episode["first_non_unreachable_s"] + 0.15
                        )
                        for timestamp in no_route_times
                    )
                ),
                raw_episodes[0],
            )
        raw_onset = primary_episode["onset_s"] if primary_episode is not None else None
        raw_recovery = (
            primary_link["first_sustained_raw_healthy_s"]
            if primary_link is not None
            else (episode_recovery(primary_episode) if primary_episode is not None else None)
        )

        sat_apply = primary_link["apply_time_s"] if primary_link is not None else None
        first_actual_satellite_route = next(
            (
                number(row.get("ApplyTime"))
                for row in satellite_policy_events
                if number(row.get("ApplyTime")) >= degradation_start - 6.0
                and flag(row.get("RouteChanged"))
            ),
            None,
        )
        if primary_link is not None:
            route_loss = primary_link["first_route_loss_s"]
            tn_recovery = primary_link["tn_recovery_apply_s"]
            interruption = primary_link["route_interruption_s"]
            anticipatory_zero_interruption = primary_link[
                "anticipatory_zero_route_interruption"
            ]
        else:
            route_loss_candidates = [
                number(row.get("ApplyTime"))
                for row in cell_route_events
                if primary_episode is not None
                and primary_episode["onset_s"] <= number(row.get("ApplyTime"))
                and (
                    primary_episode["first_non_unreachable_s"] is None
                    or number(row.get("ApplyTime"))
                    <= primary_episode["first_non_unreachable_s"] + 0.15
                )
                and row.get("ActualRouteMode") == "NO_BACKHAUL_AVAILABLE"
            ]
            route_loss = min(route_loss_candidates) if route_loss_candidates else None
            tn_event = next(
                (
                    row
                    for row in tn_policy_events
                    if raw_recovery is not None
                    and number(row.get("DecisionTime")) >= raw_recovery
                ),
                None,
            )
            tn_recovery = number(tn_event.get("ApplyTime")) if tn_event is not None else None
            interruption = None
            anticipatory_zero_interruption = False
        per_cell[str(cell)] = {
            "first_raw_unreachable_s": raw_onset,
            "first_raw_healthy_after_outage_s": raw_recovery,
            "first_satellite_apply_s": sat_apply,
            "first_actual_satellite_route_transition_s": first_actual_satellite_route,
            "first_actual_route_loss_s": route_loss,
            "route_loss_to_satellite_restore_s": interruption,
            "anticipatory_zero_route_interruption": anticipatory_zero_interruption,
            "satellite_apply_minus_degradation_start_s": (
                sat_apply - degradation_start if sat_apply is not None else None
            ),
            "satellite_apply_minus_raw_unreachable_s": (
                sat_apply - raw_onset if sat_apply is not None and raw_onset is not None else None
            ),
            "first_tn_recovery_apply_s": tn_recovery,
            "tn_recovery_minus_degradation_stop_s": (
                tn_recovery - degradation_stop if tn_recovery is not None else None
            ),
            "tn_recovery_minus_raw_healthy_s": (
                tn_recovery - raw_recovery
                if tn_recovery is not None and raw_recovery is not None
                else None
            ),
            "changed_events": len(cell_events),
            "actual_route_changed_events": len(cell_route_events),
            "raw_outage_episodes": raw_episodes,
            "linked_satellite_policy_events": linked_satellite_events,
            "unlinked_satellite_policy_events": unlinked_satellite_events,
        }
    def mean_available(field: str) -> float | None:
        values = [
            cell_metrics[field]
            for cell_metrics in per_cell.values()
            if cell_metrics[field] is not None
        ]
        return sum(values) / len(values) if values else None

    all_linked_events = [
        event
        for cell_metrics in per_cell.values()
        for event in cell_metrics["linked_satellite_policy_events"]
    ]

    def mean_linked(field: str) -> float | None:
        values = [
            event[field]
            for event in all_linked_events
            if event.get(field) is not None
        ]
        return sum(values) / len(values) if values else None

    return {
        "scheduled_event_records": len(events),
        "policy_apply_records": len(policy_events),
        "mode_changed_events": len(changed),
        "actual_route_changed_events": len(route_changed),
        "tn_to_satellite_events": sum(
            row.get("AppliedMode") == "SATELLITE_FALLBACK" for row in changed
        ),
        "satellite_to_tn_events": sum(row.get("AppliedMode") == "TN_DIRECT" for row in changed),
        "mean_actuation_delay_s": (
            sum(number(row.get("ActuationDelaySec"), 0.0) for row in policy_events)
            / len(policy_events)
            if policy_events
            else None
        ),
        "mean_satellite_apply_minus_degradation_start_s": (
            sum(event["apply_time_s"] - degradation_start for event in all_linked_events)
            / len(all_linked_events)
            if all_linked_events
            else None
        ),
        "mean_satellite_apply_minus_raw_unreachable_s": mean_linked(
            "apply_minus_raw_onset_s"
        ),
        "mean_route_loss_to_satellite_restore_s": mean_linked("route_interruption_s"),
        "mean_tn_recovery_minus_degradation_stop_s": mean_available(
            "tn_recovery_minus_degradation_stop_s"
        ),
        "mean_tn_recovery_minus_raw_healthy_s": mean_available(
            "tn_recovery_minus_raw_healthy_s"
        ),
        "linked_outage_episode_ids": sorted(
            {event["episode_id"] for event in all_linked_events}
        ),
        "linked_outage_episode_count": len(
            {event["episode_id"] for event in all_linked_events}
        ),
        "linked_satellite_policy_event_count": len(all_linked_events),
        "linked_satellite_policy_events": all_linked_events,
        "linked_events_without_sustained_raw_recovery": sum(
            event.get("first_sustained_raw_healthy_s") is None
            for event in all_linked_events
        ),
        "linked_events_without_tn_policy_recovery": sum(
            event.get("tn_recovery_apply_s") is None for event in all_linked_events
        ),
        "raw_outage_cells": sum(
            cell_metrics["first_raw_unreachable_s"] is not None
            for cell_metrics in per_cell.values()
        ),
        "raw_outage_cell_ids": [
            int(cell)
            for cell, cell_metrics in per_cell.items()
            if cell_metrics["first_raw_unreachable_s"] is not None
        ],
        "raw_recovery_cells": sum(
            cell_metrics["first_raw_healthy_after_outage_s"] is not None
            for cell_metrics in per_cell.values()
        ),
        "raw_recovery_cell_ids": [
            int(cell)
            for cell, cell_metrics in per_cell.items()
            if cell_metrics["first_raw_healthy_after_outage_s"] is not None
        ],
        "satellite_route_transition_cells": sum(
            cell_metrics["first_actual_satellite_route_transition_s"] is not None
            for cell_metrics in per_cell.values()
        ),
        "satellite_route_transition_cell_ids": [
            int(cell)
            for cell, cell_metrics in per_cell.items()
            if cell_metrics["first_actual_satellite_route_transition_s"] is not None
        ],
        "linked_satellite_policy_cells": sum(
            bool(cell_metrics["linked_satellite_policy_events"])
            for cell_metrics in per_cell.values()
        ),
        "linked_satellite_policy_cell_ids": [
            int(cell)
            for cell, cell_metrics in per_cell.items()
            if cell_metrics["linked_satellite_policy_events"]
        ],
        "unlinked_satellite_policy_events": sum(
            len(cell_metrics["unlinked_satellite_policy_events"])
            for cell_metrics in per_cell.values()
        ),
        "raw_outage_episode_count": sum(
            len(cell_metrics["raw_outage_episodes"])
            for cell_metrics in per_cell.values()
        ),
        "no_backhaul_route_transition_cells": sum(
            cell_metrics["first_actual_route_loss_s"] is not None
            for cell_metrics in per_cell.values()
        ),
        "tn_recovery_route_transition_cells": sum(
            cell_metrics["first_tn_recovery_apply_s"] is not None
            for cell_metrics in per_cell.values()
        ),
        "tn_recovery_route_transition_cell_ids": [
            int(cell)
            for cell, cell_metrics in per_cell.items()
            if cell_metrics["first_tn_recovery_apply_s"] is not None
        ],
        "per_cell": per_cell,
    }


def service_coverage_metrics(
    rsrp_path: Path,
    event_path: Path,
    xhaul_path: Path,
    expected_ues: int,
    rsrp_threshold_dbm: float,
    sim_time: float,
    degradation_start: float,
    degradation_stop: float,
    max_measurement_age_s: float = 2.5,
) -> dict[str, Any]:
    measurements: dict[str, list[tuple[float, int, float]]] = defaultdict(list)
    with rsrp_path.open(encoding="utf-8", errors="replace") as stream:
        header = stream.readline().split()
        indexes = {name: index for index, name in enumerate(header)}
        for line in stream:
            fields = line.split()
            try:
                if fields[indexes["Serving"]] not in {"1", "true", "True"}:
                    continue
                measurements[fields[indexes["IMSI"]]].append(
                    (
                        float(fields[indexes["Time"]]),
                        int(fields[indexes["CellId"]]),
                        float(fields[indexes["RSRP"]]),
                    )
                )
            except (IndexError, KeyError, ValueError):
                continue
    for values in measurements.values():
        values.sort()

    mode_history: dict[int, list[tuple[float, str]]] = defaultdict(list)
    for row in read_csv_rows(xhaul_path):
        cell = integer(row.get("UavCellId"))
        if cell not in mode_history:
            mode_history[cell].append((-math.inf, "TN_DIRECT"))
    for row in read_csv_rows(event_path):
        if flag(row.get("RouteChanged")):
            mode_history[integer(row.get("UavCellId"))].append(
                (number(row.get("ApplyTime")), row.get("ActualRouteMode", ""))
            )
    for values in mode_history.values():
        values.sort()
    uav_cells = set(mode_history)

    def onward_usable(cell: int, timestamp: float) -> bool:
        if cell not in uav_cells:
            return True
        history = mode_history[cell]
        index = bisect.bisect_right([item[0] for item in history], timestamp) - 1
        return index >= 0 and history[index][1] in {"TN_DIRECT", "SATELLITE_FALLBACK"}

    first_grid_time = 10
    samples: list[tuple[int, int]] = []
    for timestamp in range(first_grid_time, int(math.floor(sim_time)) + 1):
        covered = 0
        for history in measurements.values():
            index = bisect.bisect_right([item[0] for item in history], timestamp) - 1
            if index < 0:
                continue
            measured_at, cell, rsrp = history[index]
            if (
                timestamp - measured_at <= max_measurement_age_s
                and rsrp >= rsrp_threshold_dbm
                and onward_usable(cell, timestamp)
            ):
                covered += 1
        samples.append((timestamp, covered))

    windows = {
        "full": (first_grid_time, sim_time, True),
        "pre_degradation": (first_grid_time, degradation_start, False),
        "degradation": (degradation_start, degradation_stop, False),
        "post_degradation": (degradation_stop, sim_time, True),
    }
    summaries: dict[str, Any] = {}
    for name, (start, stop, include_stop) in windows.items():
        selected = [
            covered
            for timestamp, covered in samples
            if timestamp >= start and (timestamp <= stop if include_stop else timestamp < stop)
        ]
        numerator = sum(selected)
        denominator = expected_ues * len(selected)
        summaries[name] = {
            "covered_ue_seconds": numerator,
            "denominator_ue_seconds": denominator,
            "rate": rate(numerator, denominator),
            "time_samples": len(selected),
        }
    return {
        "method": (
            "1 s grid; latest serving-cell RSRP no older than 2.5 s; RSRP >= threshold; "
            "UAV serving cells require TN_DIRECT or SATELLITE_FALLBACK; component windows "
            "are disjoint [start,stop), with final simulation stop inclusive"
        ),
        "is_proxy": True,
        "rsrp_threshold_dbm": rsrp_threshold_dbm,
        "expected_ues_denominator": expected_ues,
        "observed_imsi_count": len(measurements),
        "windows": summaries,
    }


def uav_exposure_and_recovery_metrics(
    identity_path: Path,
    rsrp_path: Path,
    xhaul_path: Path,
    event_path: Path,
    packet_path: Path,
    degradation_start: float,
    degradation_stop: float,
    max_measurement_age_s: float = 2.5,
) -> dict[str, Any]:
    ue_to_imsi = {
        integer(row.get("UE")): row.get("IMSI", "")
        for row in read_csv_rows(identity_path)
        if row.get("Group") == "S1"
    }
    measurements: dict[str, list[tuple[float, int]]] = defaultdict(list)
    with rsrp_path.open(encoding="utf-8", errors="replace") as stream:
        header = stream.readline().split()
        indexes = {name: index for index, name in enumerate(header)}
        for line in stream:
            fields = line.split()
            try:
                if fields[indexes["Serving"]] not in {"1", "true", "True"}:
                    continue
                measurements[fields[indexes["IMSI"]]].append(
                    (float(fields[indexes["Time"]]), int(fields[indexes["CellId"]]))
                )
            except (IndexError, KeyError, ValueError):
                continue
    for values in measurements.values():
        values.sort()
    uav_cells = {integer(row.get("UavCellId")) for row in read_csv_rows(xhaul_path)}

    def serving_cell(ue: int, timestamp: float) -> int | None:
        history = measurements.get(ue_to_imsi.get(ue, ""), [])
        if not history:
            return None
        times = [item[0] for item in history]
        index = bisect.bisect_right(times, timestamp) - 1
        if index < 0 or timestamp - history[index][0] > max_measurement_age_s:
            return None
        return history[index][1]

    packet_rows = read_csv_rows(packet_path)
    first_tx_by_ue: dict[int, float] = {}
    for row in packet_rows:
        if row.get("Event") != "TX":
            continue
        ue = integer(row.get("UE"), -1)
        timestamp = number(row.get("Time"))
        if ue not in ue_to_imsi or not math.isfinite(timestamp):
            continue
        first_tx_by_ue[ue] = min(timestamp, first_tx_by_ue.get(ue, math.inf))

    first_serving_by_ue: dict[int, tuple[float, int]] = {}
    for ue, imsi in ue_to_imsi.items():
        history = measurements.get(imsi, [])
        if history:
            first_serving_by_ue[ue] = history[0]
    attached_before_first_tx = {
        ue
        for ue, first_tx in first_tx_by_ue.items()
        if ue in first_serving_by_ue
        and first_serving_by_ue[ue][0] <= first_tx + 1e-9
    }
    initial_uav_served_ues = {
        ue
        for ue, (_timestamp, cell) in first_serving_by_ue.items()
        if cell in uav_cells
    }
    pre_event_tx_ues = {
        integer(row.get("UE"))
        for row in packet_rows
        if row.get("Event") == "TX"
        and 10.0 <= number(row.get("Time")) < degradation_start
    }
    pre_event_rx_ues = {
        integer(row.get("UE"))
        for row in packet_rows
        if row.get("Event") == "RX"
        and 10.0 <= number(row.get("SenderTimestampSec")) < degradation_start
    }
    pre_event_offered_ues_by_direction = {
        direction: {
            integer(row.get("UE"))
            for row in packet_rows
            if row.get("Event") == "TX"
            and row.get("Dir") == direction
            and 10.0 <= number(row.get("Time")) < degradation_start
        }
        for direction in ("DL", "UL")
    }
    pre_event_received_ues_by_direction = {
        direction: {
            integer(row.get("UE"))
            for row in packet_rows
            if row.get("Event") == "RX"
            and row.get("Dir") == direction
            and 10.0 <= number(row.get("SenderTimestampSec")) < degradation_start
        }
        for direction in ("DL", "UL")
    }
    tx_rows = [
        row
        for row in packet_rows
        if row.get("Event") == "TX"
        and degradation_start <= number(row.get("Time")) < degradation_stop
    ]
    uav_tx_rows = [
        row
        for row in tx_rows
        if serving_cell(integer(row.get("UE")), number(row.get("Time"))) in uav_cells
    ]
    observed_tx_rows = [
        row
        for row in tx_rows
        if serving_cell(integer(row.get("UE")), number(row.get("Time"))) is not None
    ]
    observed_tx_ues = {integer(row.get("UE")) for row in observed_tx_rows}
    uav_tx_ues = {integer(row.get("UE")) for row in uav_tx_rows}

    rx_rows_by_ue: dict[int, list[tuple[float, float]]] = defaultdict(list)
    for row in packet_rows:
        if row.get("Event") == "RX":
            rx_rows_by_ue[integer(row.get("UE"))].append(
                (number(row.get("Time")), number(row.get("SenderTimestampSec")))
            )
    for values in rx_rows_by_ue.values():
        values.sort()

    # A valid forward predictor may apply satellite before the registered
    # impairment onset. Include the complete 6 s prediction horizon so packet
    # recovery is checked for anticipatory as well as reactive transitions.
    recovery_window_start = max(10.0, degradation_start - 6.0)
    recovery_events: list[dict[str, Any]] = []
    for row in read_csv_rows(event_path):
        if not flag(row.get("RouteChanged")) or row.get("ActualRouteMode") != "SATELLITE_FALLBACK":
            continue
        apply_time = number(row.get("ApplyTime"))
        cell = integer(row.get("UavCellId"))
        if not (recovery_window_start <= apply_time <= degradation_stop):
            continue
        affected_ues = [
            ue for ue in ue_to_imsi if serving_cell(ue, apply_time) == cell
        ]
        first_rx: float | None = None
        for ue in affected_ues:
            values = rx_rows_by_ue.get(ue, [])
            index = bisect.bisect_left(values, (apply_time, -math.inf))
            while index < len(values):
                receive_time, sender_time = values[index]
                if sender_time >= apply_time and serving_cell(ue, receive_time) == cell:
                    if first_rx is None or receive_time < first_rx:
                        first_rx = receive_time
                    break
                index += 1
        recovery_events.append(
            {
                "uav_cell_id": cell,
                "route_apply_s": apply_time,
                "affected_monitored_ues": len(affected_ues),
                "first_successful_packet_rx_s": first_rx,
                "route_to_first_successful_packet_s": (
                    first_rx - apply_time if first_rx is not None else None
                ),
            }
        )

    association_fraction = rate(len(observed_tx_rows), len(tx_rows))
    fraction = rate(len(uav_tx_rows), len(tx_rows))
    return {
        "window": [degradation_start, degradation_stop],
        "satellite_recovery_event_window": [recovery_window_start, degradation_stop],
        "uav_cells": sorted(uav_cells),
        "monitored_ue_count": len(ue_to_imsi),
        "monitored_ues_with_packet_offers": len(first_tx_by_ue),
        "attached_before_first_packet_tx_count": len(attached_before_first_tx),
        "missing_attachment_before_first_packet_tx_ues": sorted(
            set(ue_to_imsi) - attached_before_first_tx
        ),
        "first_serving_measurement_count": len(first_serving_by_ue),
        "missing_first_serving_measurement_ues": sorted(
            set(ue_to_imsi) - set(first_serving_by_ue)
        ),
        "initial_uav_served_monitored_ues": len(initial_uav_served_ues),
        "initial_uav_served_ue_indexes": sorted(initial_uav_served_ues),
        "pre_event_offered_ue_count": len(pre_event_tx_ues & set(ue_to_imsi)),
        "pre_event_received_ue_count": len(pre_event_rx_ues & set(ue_to_imsi)),
        "missing_pre_event_offered_ues": sorted(set(ue_to_imsi) - pre_event_tx_ues),
        "missing_pre_event_received_ues": sorted(set(ue_to_imsi) - pre_event_rx_ues),
        "pre_event_offered_ue_count_by_direction": {
            direction: len(values & set(ue_to_imsi))
            for direction, values in pre_event_offered_ues_by_direction.items()
        },
        "pre_event_received_ue_count_by_direction": {
            direction: len(values & set(ue_to_imsi))
            for direction, values in pre_event_received_ues_by_direction.items()
        },
        "missing_pre_event_received_ues_by_direction": {
            direction: sorted(set(ue_to_imsi) - values)
            for direction, values in pre_event_received_ues_by_direction.items()
        },
        "offered_tx_attempts": len(tx_rows),
        "tx_attempts_with_fresh_serving_measurement": len(observed_tx_rows),
        "fresh_serving_measurement_fraction": association_fraction,
        "monitored_ues_with_fresh_serving_measurement": len(
            observed_tx_ues & set(ue_to_imsi)
        ),
        "missing_fresh_serving_measurement_ues": sorted(
            set(ue_to_imsi) - observed_tx_ues
        ),
        "uav_served_tx_attempts": len(uav_tx_rows),
        "uav_served_tx_attempt_fraction": fraction,
        "distinct_uav_served_monitored_ues": len(uav_tx_ues & set(ue_to_imsi)),
        "distinct_uav_served_ue_indexes": sorted(uav_tx_ues & set(ue_to_imsi)),
        "minimum_required_fresh_serving_measurement_fraction": 0.80,
        "minimum_required_uav_tx_attempt_fraction": 0.05,
        "recovery_events": recovery_events,
    }


def pre_event_xhaul_route_health(
    xhaul_path: Path,
    event_path: Path,
    degradation_start: float,
    sim_time: float,
) -> dict[str, Any]:
    """Verify that every UAV starts with usable TN onward routing.

    The route is reconstructed from route-change events rather than inferred
    from the controller's requested mode.  This prevents an early satellite
    failover from contaminating the nominal pre-event access baseline.
    """
    window_start = 10.0
    window_stop = min(degradation_start, sim_time)
    xhaul_rows = read_csv_rows(xhaul_path)
    event_rows = read_csv_rows(event_path)
    cells = sorted({integer(row.get("UavCellId")) for row in xhaul_rows})
    route_history: dict[int, list[tuple[float, str]]] = {
        cell: [(-math.inf, "TN_DIRECT")] for cell in cells
    }
    for row in event_rows:
        if not flag(row.get("RouteChanged")):
            continue
        cell = integer(row.get("UavCellId"))
        route_history.setdefault(cell, [(-math.inf, "TN_DIRECT")]).append(
            (number(row.get("ApplyTime")), row.get("ActualRouteMode", ""))
        )
    for history in route_history.values():
        history.sort()

    def route_at(cell: int, timestamp: float) -> str:
        observed = "TN_DIRECT"
        for apply_time, mode in route_history.get(cell, [(-math.inf, "TN_DIRECT")]):
            if apply_time > timestamp:
                break
            observed = mode
        return observed

    selected = [
        row
        for row in xhaul_rows
        if window_start <= number(row.get("Time")) < window_stop
    ]
    rows_by_cell = Counter(integer(row.get("UavCellId")) for row in selected)
    unhealthy_rows = [
        {
            "time_s": number(row.get("Time")),
            "uav_cell_id": integer(row.get("UavCellId")),
            "raw_state": row.get("RawXhaulState"),
            "connected": row.get("XhaulConnected"),
        }
        for row in selected
        if row.get("RawXhaulState") != "HEALTHY" or not flag(row.get("XhaulConnected"))
    ]
    non_tn_route_rows = [
        {
            "time_s": number(row.get("Time")),
            "uav_cell_id": integer(row.get("UavCellId")),
            "trace_actual_route_mode": row.get("ActualRouteMode"),
            "reconstructed_actual_route_mode": route_at(
                integer(row.get("UavCellId")), number(row.get("Time"))
            ),
        }
        for row in selected
        if row.get("ActualRouteMode") != "TN_DIRECT"
        or route_at(integer(row.get("UavCellId")), number(row.get("Time")))
        != "TN_DIRECT"
    ]
    early_satellite_policy_events = [
        {
            "decision_time_s": number(row.get("DecisionTime")),
            "apply_time_s": number(row.get("ApplyTime")),
            "uav_cell_id": integer(row.get("UavCellId")),
            "applied_mode": row.get("AppliedMode"),
            "actual_route_mode": row.get("ActualRouteMode"),
        }
        for row in event_rows
        if row.get("EventType", "POLICY_APPLY") == "POLICY_APPLY"
        and number(row.get("ApplyTime")) < degradation_start
        and (
            row.get("AppliedMode") == "SATELLITE_FALLBACK"
            or row.get("ActualRouteMode") == "SATELLITE_FALLBACK"
        )
    ]
    missing_cells = [cell for cell in cells if rows_by_cell[cell] == 0]
    passed = (
        window_stop > window_start
        and bool(cells)
        and not missing_cells
        and not unhealthy_rows
        and not non_tn_route_rows
        and not early_satellite_policy_events
    )
    return {
        "passed": passed,
        "window_s": [window_start, window_stop],
        "uav_cell_ids": cells,
        "sample_rows_by_cell": {str(cell): rows_by_cell[cell] for cell in cells},
        "missing_uav_cells": missing_cells,
        "unhealthy_row_count": len(unhealthy_rows),
        "first_unhealthy_rows": unhealthy_rows[:20],
        "non_tn_route_row_count": len(non_tn_route_rows),
        "first_non_tn_route_rows": non_tn_route_rows[:20],
        "early_satellite_policy_event_count": len(early_satellite_policy_events),
        "first_early_satellite_policy_events": early_satellite_policy_events[:20],
        "route_reconstruction": "TN_DIRECT initially; apply only RouteChanged events",
    }


def service_satisfaction_metrics(
    qos_path: Path,
    monitored_ues: int,
    sim_time: float,
    degradation_start: float,
    degradation_stop: float,
) -> dict[str, Any]:
    """Evaluate the preregistered bidirectional per-UE service indicator.

    A UE passes a one-second sample only when both its DL and UL rows meet the
    direction-specific rate floor and the common delay/PDR limits.  Missing or
    zero-transmission rows fail rather than silently leaving the denominator.
    """
    thresholds = {
        "DL": {"min_rate_mbps": 0.18, "max_delay_s": 0.30, "min_pdr": 0.90},
        "UL": {"min_rate_mbps": 0.045, "max_delay_s": 0.30, "min_pdr": 0.90},
    }
    rows = read_csv_rows(qos_path)
    by_key = {
        (number(row.get("Time")), integer(row.get("UE")), row.get("Dir", "")): row
        for row in rows
        if 10.0 < number(row.get("Time")) <= sim_time
    }
    sample_times = sorted({key[0] for key in by_key})
    indicators: list[tuple[float, int]] = []
    for timestamp in sample_times:
        for ue in range(monitored_ues):
            passed = True
            for direction in ("DL", "UL"):
                row = by_key.get((timestamp, ue, direction))
                if row is None:
                    passed = False
                    continue
                tx = integer(row.get("TxPackets"))
                rx = integer(row.get("RxPackets"))
                interval = number(row.get("IntervalSec"))
                rx_bytes = integer(row.get("RxBytes"))
                delay_sum = number(row.get("DelaySumSec"), 0.0)
                observed_rate = (
                    rx_bytes * 8.0 / interval / 1e6 if interval > 0.0 else 0.0
                )
                observed_pdr = rate(rx, tx) if tx > 0 else None
                observed_delay = rate(delay_sum, rx) if rx > 0 else None
                criterion = thresholds[direction]
                passed &= (
                    tx > 0
                    and observed_pdr is not None
                    and observed_delay is not None
                    and observed_rate >= criterion["min_rate_mbps"]
                    and observed_delay <= criterion["max_delay_s"]
                    and observed_pdr >= criterion["min_pdr"]
                )
            indicators.append((timestamp, int(passed)))

    windows = {
        "full": (10.0, sim_time, True),
        "pre_degradation": (10.0, degradation_start, False),
        "degradation": (degradation_start, degradation_stop, False),
        "post_degradation": (degradation_stop, sim_time, True),
    }
    summaries: dict[str, Any] = {}
    for name, (start, stop, include_stop) in windows.items():
        selected = [
            value
            for timestamp, value in indicators
            if timestamp >= start and (timestamp <= stop if include_stop else timestamp < stop)
        ]
        summaries[name] = {
            "passing_ue_samples": sum(selected),
            "denominator_ue_samples": len(selected),
            "rate": rate(sum(selected), len(selected)),
            "time_samples": len({
                timestamp
                for timestamp, _ in indicators
                if timestamp >= start and (timestamp <= stop if include_stop else timestamp < stop)
            }),
        }
    return {
        "method": "1 s per-UE samples; both DL and UL criteria must pass",
        "warmup_excluded_through_s": 10.0,
        "monitored_ues_denominator": monitored_ues,
        "thresholds": thresholds,
        "windows": summaries,
    }


def validate_onnx_rows(
    ai_trace: Path,
    model_path: Path,
    tolerance: float,
    required: bool,
) -> tuple[dict[str, Any], dict[str, Any]]:
    rows = read_csv_rows(ai_trace)
    valid_rows = [row for row in rows if flag(row.get("FeatureValid"))]
    inference_rows = [row for row in valid_rows if flag(row.get("InferenceOk"))]
    if not required:
        details = {
            "applicable": False,
            "valid_feature_rows": len(valid_rows),
            "inference_rows": len(inference_rows),
        }
        return gate("onnx_row_probability_parity", True, **details), details

    try:
        import numpy as np
        import onnxruntime as ort
    except ImportError as error:
        details = {"applicable": True, "error": str(error)}
        return gate("onnx_row_probability_parity", False, **details), details

    session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    model_inputs = session.get_inputs()
    model_outputs = session.get_outputs()
    input_name = model_inputs[0].name
    output_names = [output.name for output in model_outputs]
    probability_name = "probabilities" if "probabilities" in output_names else output_names[-1]
    model_interface = {
        "inputs": [
            {"name": value.name, "shape": value.shape, "type": value.type}
            for value in model_inputs
        ],
        "outputs": [
            {"name": value.name, "shape": value.shape, "type": value.type}
            for value in model_outputs
        ],
    }
    model_interface_ok = (
        len(model_inputs) == 1
        and model_inputs[0].name == "features"
        and list(model_inputs[0].shape) == [None, len(FEATURE_COLUMNS)]
        and model_inputs[0].type == "tensor(float)"
        and output_names == ["label", "probabilities"]
        and list(model_outputs[1].shape) == [None, 2]
        and model_outputs[1].type == "tensor(float)"
    )
    mismatches: list[dict[str, Any]] = []
    maximum_error = 0.0
    model_hash_rows_ok = True
    schema_rows_ok = True
    for index, row in enumerate(valid_rows):
        if not flag(row.get("InferenceOk")):
            mismatches.append({"row": index, "reason": "valid feature row has no inference"})
            continue
        try:
            features = np.asarray([[float(row[column]) for column in FEATURE_COLUMNS]], dtype=np.float32)
            outputs = session.run([probability_name], {input_name: features})
            replayed = float(np.asarray(outputs[0])[0, 1])
            recorded = float(row["PredictionProbability"])
            error = abs(replayed - recorded)
            maximum_error = max(maximum_error, error)
            threshold = float(row["Threshold"])
            raw_prediction = integer(row.get("RawPrediction"))
            if error > tolerance or raw_prediction != int(replayed >= threshold):
                mismatches.append(
                    {
                        "row": index,
                        "time": row.get("Time"),
                        "cell": row.get("UavCellId"),
                        "recorded": recorded,
                        "replayed": replayed,
                        "absolute_error": error,
                        "recorded_raw_prediction": raw_prediction,
                        "replayed_raw_prediction": int(replayed >= threshold),
                    }
                )
        except (KeyError, TypeError, ValueError, IndexError) as error:
            mismatches.append({"row": index, "reason": str(error)})
        model_hash_rows_ok &= row.get("ModelSha256") == MODEL_SHA256
        schema_rows_ok &= row.get("FeatureSchemaVersion") == "uav-sustained-outage-v1"

    details = {
        "applicable": True,
        "valid_feature_rows": len(valid_rows),
        "inference_rows": len(inference_rows),
        "probability_tolerance": tolerance,
        "maximum_absolute_error": maximum_error,
        "mismatch_count": len(mismatches),
        "first_mismatches": mismatches[:10],
        "model_hash_rows_ok": model_hash_rows_ok,
        "schema_version_rows_ok": schema_rows_ok,
        "model_interface": model_interface,
        "model_interface_ok": model_interface_ok,
        "ordered_trace_feature_columns": list(FEATURE_COLUMNS),
    }
    passed = (
        bool(valid_rows)
        and not mismatches
        and model_hash_rows_ok
        and schema_rows_ok
        and model_interface_ok
    )
    return gate("onnx_row_probability_parity", passed, **details), details


def validate_online_feature_construction(
    ai_trace: Path,
    xhaul_trace: Path,
    expected_cadence_s: float = 2.0,
    cadence_tolerance_s: float = 1e-6,
    expected_first_epoch_s: float = 4.5,
    expected_last_epoch_s: float = 28.5,
) -> dict[str, Any]:
    """Independently reconstruct the eight causal C++ features from raw rows."""
    ai_source_rows = read_csv_rows(ai_trace)
    xhaul_source_rows = read_csv_rows(xhaul_trace)

    def trace_key(row: dict[str, str]) -> tuple[float, int]:
        return (round(number(row.get("Time")), 9), integer(row.get("UavCellId")))

    ai_key_counts = Counter(trace_key(row) for row in ai_source_rows)
    xhaul_key_counts = Counter(trace_key(row) for row in xhaul_source_rows)
    ai_rows = {trace_key(row): row for row in ai_source_rows}
    by_cell: dict[int, list[dict[str, str]]] = defaultdict(list)
    ai_by_cell: dict[int, list[dict[str, str]]] = defaultdict(list)
    for row in xhaul_source_rows:
        by_cell[integer(row.get("UavCellId"))].append(row)
    for row in ai_source_rows:
        ai_by_cell[integer(row.get("UavCellId"))].append(row)
    mismatches: list[dict[str, Any]] = []
    checked = 0

    def record_mismatch(detail: dict[str, Any]) -> None:
        if len(mismatches) < 20:
            mismatches.append(detail)

    def f32(value: float) -> float:
        return struct.unpack("!f", struct.pack("!f", float(value)))[0]

    cadence_errors: list[dict[str, Any]] = []
    cadence_error_count = 0
    expected_epochs: list[float] = []
    epoch = expected_first_epoch_s
    while epoch <= expected_last_epoch_s + cadence_tolerance_s:
        expected_epochs.append(round(epoch, 9))
        epoch += expected_cadence_s
    epoch_contract_errors: list[dict[str, Any]] = []
    for trace_name, grouped_rows in (("xhaul", by_cell), ("ai", ai_by_cell)):
        if set(grouped_rows) != set(EXPECTED_UAV_CELL_IDS):
            epoch_contract_errors.append(
                {
                    "trace": trace_name,
                    "reason": "UAV cell set mismatch",
                    "expected_cells": list(EXPECTED_UAV_CELL_IDS),
                    "observed_cells": sorted(grouped_rows),
                }
            )
        for cell, cell_rows in grouped_rows.items():
            timestamps = sorted(number(row.get("Time")) for row in cell_rows)
            rounded_timestamps = [round(timestamp, 9) for timestamp in timestamps]
            if rounded_timestamps != expected_epochs:
                epoch_contract_errors.append(
                    {
                        "trace": trace_name,
                        "cell": cell,
                        "reason": "control epoch sequence mismatch",
                        "expected": expected_epochs,
                        "observed": rounded_timestamps,
                    }
                )
            for previous, current in zip(timestamps, timestamps[1:]):
                gap = current - previous
                if (
                    not math.isfinite(previous)
                    or not math.isfinite(current)
                    or abs(gap - expected_cadence_s) > cadence_tolerance_s
                ):
                    cadence_error_count += 1
                    if len(cadence_errors) < 20:
                        cadence_errors.append(
                            {
                                "trace": trace_name,
                                "cell": cell,
                                "previous_time": previous,
                                "current_time": current,
                                "gap_s": gap,
                            }
                        )

    duplicate_ai_keys = [key for key, count in ai_key_counts.items() if count != 1]
    duplicate_xhaul_keys = [key for key, count in xhaul_key_counts.items() if count != 1]
    missing_ai_keys = sorted(set(xhaul_key_counts) - set(ai_key_counts))
    extra_ai_keys = sorted(set(ai_key_counts) - set(xhaul_key_counts))

    for cell, rows in by_cell.items():
        rows.sort(key=lambda item: number(item.get("Time")))
        for index, row in enumerate(rows):
            timestamp = number(row.get("Time"))
            ai = ai_rows.get((round(timestamp, 9), cell))
            if ai is None:
                record_mismatch({"cell": cell, "time": timestamp, "reason": "missing AI row"})
                continue
            connected = flag(row.get("XhaulConnected"))
            expected_valid = index >= 2
            if expected_valid:
                previous = rows[index - 1]
                previous2 = rows[index - 2]
                expected_valid = (
                    abs(timestamp - number(previous.get("Time")) - expected_cadence_s)
                    <= cadence_tolerance_s
                    and abs(
                        number(previous.get("Time"))
                        - number(previous2.get("Time"))
                        - expected_cadence_s
                    )
                    <= cadence_tolerance_s
                )
            if flag(ai.get("FeatureValid")) != expected_valid:
                record_mismatch(
                    {"cell": cell, "time": timestamp, "reason": "FeatureValid mismatch"}
                )
                continue
            if not expected_valid:
                continue
            previous = rows[index - 1]
            previous2 = rows[index - 2]
            previous_connected = flag(previous.get("XhaulConnected"))
            previous2_connected = flag(previous2.get("XhaulConnected"))
            distance = number(row.get("BestDonorDistanceM")) if connected else -1.0
            variation = number(row.get("XhaulChannelVariationDb")) if connected else 0.0
            rsrp = number(row.get("XhaulRsrpDbm")) if connected else -999.0
            previous_distance = (
                number(previous.get("BestDonorDistanceM")) if previous_connected else -1.0
            )
            previous_rsrp = number(previous.get("XhaulRsrpDbm")) if previous_connected else -999.0
            previous2_rsrp = number(previous2.get("XhaulRsrpDbm")) if previous2_connected else -999.0
            expected = (
                distance,
                1.0 if connected else 0.0,
                variation,
                rsrp,
                number(row.get("RawXhaulStateAgeSec")),
                distance - previous_distance if connected and previous_connected else 0.0,
                rsrp - previous_rsrp if connected and previous_connected else 0.0,
                rsrp - previous2_rsrp if connected and previous2_connected else 0.0,
            )
            for column, expected_value in zip(FEATURE_COLUMNS, expected):
                observed = number(ai.get(column))
                expected_float = f32(expected_value)
                checked += 1
                tolerance = max(1e-6, abs(expected_float) * 2e-7)
                if not math.isfinite(observed) or abs(observed - expected_float) > tolerance:
                    record_mismatch(
                        {
                            "cell": cell,
                            "time": timestamp,
                            "feature": column,
                            "expected_float32": expected_float,
                            "observed": observed,
                        }
                    )
    structural_ok = not any(
        (
            cadence_errors,
            epoch_contract_errors,
            duplicate_ai_keys,
            duplicate_xhaul_keys,
            missing_ai_keys,
            extra_ai_keys,
        )
    )
    return gate(
        "independent_online_feature_reconstruction",
        checked > 0 and structural_ok and not mismatches,
        expected_cadence_s=expected_cadence_s,
        cadence_tolerance_s=cadence_tolerance_s,
        cadence_error_count=cadence_error_count,
        first_cadence_errors=cadence_errors,
        expected_control_epochs_s=expected_epochs,
        control_epoch_contract_error_count=len(epoch_contract_errors),
        first_control_epoch_contract_errors=epoch_contract_errors[:20],
        duplicate_ai_key_count=len(duplicate_ai_keys),
        first_duplicate_ai_keys=duplicate_ai_keys[:20],
        duplicate_xhaul_key_count=len(duplicate_xhaul_keys),
        first_duplicate_xhaul_keys=duplicate_xhaul_keys[:20],
        missing_ai_row_count=len(missing_ai_keys),
        first_missing_ai_keys=missing_ai_keys[:20],
        extra_ai_row_count=len(extra_ai_keys),
        first_extra_ai_keys=extra_ai_keys[:20],
        checked_feature_values=checked,
        mismatch_count=len(mismatches),
        first_mismatches=mismatches,
    )


def verify_artifact_manifest(meta_dir: Path) -> tuple[bool, dict[str, Any]]:
    manifest = meta_dir / "raw-artifacts.sha256"
    failures: list[dict[str, str]] = []
    checked = 0
    if not manifest.is_file():
        return False, {"error": f"missing {manifest}"}
    with manifest.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if not line.strip():
                continue
            expected, filename = line.rstrip("\n").split(maxsplit=1)
            filename = filename.lstrip(" *")
            path = Path(filename)
            checked += 1
            if not path.is_file():
                failures.append({"path": filename, "reason": "missing"})
            else:
                actual = sha256(path)
                if actual != expected:
                    failures.append({"path": filename, "expected": expected, "actual": actual})
    return checked > 0 and not failures, {"checked_files": checked, "failures": failures[:20]}


def validate_run(meta_dir: Path, probability_tolerance: float, pingpong_window_s: float) -> dict[str, Any]:
    metadata = read_json(meta_dir / "run.json")
    status = read_json(meta_dir / "process-status.json")
    result_dir = Path(metadata["result_dir"])
    sim_time = float(metadata["sim_time_s"])
    degradation_start = float(metadata["degradation_start_s"])
    degradation_stop = float(metadata["degradation_stop_s"])
    policy = str(metadata["policy"])
    scope = str(metadata.get("scope", ""))
    is_smoke = scope == "smoke"
    paths = {
        "xhaul": result_dir / "xhaul-autonomy-trace.csv",
        "ai": result_dir / "uav-switching-ai-trace.csv",
        "events": result_dir / "uav-switching-events.csv",
        "qos": result_dir / "qos-vs-time.txt",
        "satellite": result_dir / "sat-backhaul-trace.txt",
        "rsrp": result_dir / "rsrp-trace.tr",
        "handover": result_dir / "handover-trace.tr",
        "handover_failure": result_dir / "handover-failure-trace.tr",
        "ues1_position": result_dir / "ues1-position-trace.tr",
        "ues2_position": result_dir / "ues2-position-trace.tr",
        "uav_position": result_dir / "uav-position-trace.tr",
        "final_flow": result_dir / "final-flow-report.txt",
        "identity": result_dir / "ue-identity-map.csv",
        "packets": result_dir / "monitored-packet-events.csv",
    }
    gates: list[dict[str, Any]] = []
    gates.append(gate("process_exit_zero", status.get("exit_code") == 0, status=status))
    required = [
        "xhaul",
        "ai",
        "events",
        "qos",
        "rsrp",
        "ues1_position",
        "ues2_position",
        "uav_position",
        "final_flow",
        "identity",
        "packets",
    ]
    if policy != "no-satellite":
        required.append("satellite")
    missing = [name for name in required if not paths[name].is_file() or paths[name].stat().st_size == 0]
    gates.append(gate("required_raw_artifacts", not missing, missing=missing))
    event_trace_exists = {
        name: paths[name].is_file() for name in ("handover", "handover_failure")
    }
    gates.append(
        gate("handover_event_trace_files", all(event_trace_exists.values()), **event_trace_exists)
    )

    model_path = Path(metadata["model_path"])
    schema_path = Path(metadata["schema_path"])
    identity_checks = {
        "source": Path(metadata["source_path"]).is_file()
        and sha256(Path(metadata["source_path"])) == metadata["source_sha256"],
        "binary": Path(metadata["binary_path"]).is_file()
        and sha256(Path(metadata["binary_path"])) == metadata["binary_sha256"],
        "model": model_path.is_file()
        and sha256(model_path) == MODEL_SHA256
        and metadata.get("model_sha256") == MODEL_SHA256,
        "schema": schema_path.is_file()
        and sha256(schema_path) == SCHEMA_SHA256
        and metadata.get("schema_sha256") == SCHEMA_SHA256,
    }
    for path_field, hash_field in PROVENANCE_PATH_HASH_FIELDS:
        artifact_path = Path(str(metadata.get(path_field, "")))
        expected_hash = str(metadata.get(hash_field, ""))
        identity_checks[hash_field] = (
            bool(metadata.get(path_field))
            and bool(re.fullmatch(r"[0-9a-f]{64}", expected_hash))
            and artifact_path.is_file()
            and sha256(artifact_path) == expected_hash
        )
    gates.append(gate("frozen_artifact_identity", all(identity_checks.values()), **identity_checks))
    expected_radio_contract = {
        "rlc_mode": "um",
        "harq_retransmissions_enabled": True,
        "dedicated_qos_flows_enabled": True,
        "use_rsrp_lm": False,
        "fixed_initial_association": True,
    }
    gates.append(
        gate(
            "frozen_radio_contract",
            metadata.get("radio_contract") == expected_radio_contract,
            expected=expected_radio_contract,
            observed=metadata.get("radio_contract"),
        )
    )
    expected_frozen_design = {
        "monitored_ues": 50,
        "background_ues": 50,
        "tn_gnbs": 4,
        "uav_gnbs": 3,
        "max_ues_per_tn_gnb": 10,
        "max_ues_per_uav_gnb": 30,
        "tn_total_capacity": 40,
        "uav_total_capacity": 90,
        "ground_attach_delay_s": 5.0,
        "initial_attach_stagger_s": 0.25,
        "monitored_dl_start_s": 6.0,
        "monitored_ul_start_s": 6.5,
        "deterministic_access_anchors": True,
        "explicit_initial_anchor_attachment": True,
        "ue_area_half_w_m": 10000.0,
        "ue_area_half_h_m": 5000.0,
        "uav_initial_area_half_w_m": 1000.0,
        "uav_initial_area_half_h_m": 800.0,
        "uav_area_half_w_m": 3000.0,
        "uav_area_half_h_m": 1500.0,
        "uav_speed_mps": 0.0,
        "mobility_update_ms": 200.0,
        "monitored_ue_speed_mps": 5.0,
        "background_ue_speed_mps": 4.0,
        "access_mobility_description": "stationary UAV anchors with bounded local UE mobility",
        "xhaul_pathloss_exponent": 4.5,
        "ue_placement_mode": "capacity-stratified-clustered",
        "capacity_stratified_ue_placement": True,
        "ue_cluster_radius_m": 25.0,
        "expected_monitored_tn_ues": 40,
        "expected_monitored_uav_ues": 10,
        "capacity_stratified_exposure": True,
    }
    gates.append(
        gate(
            "frozen_capacity_stratified_design",
            metadata.get("frozen_design") == expected_frozen_design
            and metadata.get("expected_ues") == 100
            and metadata.get("monitored_ues") == 50,
            expected=expected_frozen_design,
            observed=metadata.get("frozen_design"),
            expected_ues=metadata.get("expected_ues"),
            monitored_ues=metadata.get("monitored_ues"),
        )
    )
    expected_deployment_mode = "tn-uav-satellite"
    expected_ue_placement = {
        "mode": "capacity-stratified-clustered",
        "capacity_stratified": True,
        "cluster_radius_m": 25.0,
        "global_area_half_w_m": 10000.0,
        "global_area_half_h_m": 5000.0,
    }
    expected_ue_mobility_contract = {
        "description": "stationary UAV anchors with bounded local UE mobility",
        "monitored_ue_speed_mps": 5.0,
        "background_ue_speed_mps": 4.0,
        "uav_speed_mps": 0.0,
        "update_interval_ms": 200.0,
        "cluster_half_width_m": 25.0,
        "cluster_half_height_m": 25.0,
    }
    gates.append(
        gate(
            "frozen_evidence_scope",
            metadata.get("protocol_version") == "uav-reviewer-integration-v10"
            and metadata.get("evidence_scope")
            == "one-seed backhaul policy integration and wall-time gate under fixed initial association"
            and metadata.get("evidence_status")
            == "integration-only; not manuscript efficacy evidence"
            and metadata.get("statistical_scope")
            == "single matched RngRun=6 pair; no confidence interval or generalization claim"
            and metadata.get("rng_seed") == 1
            and metadata.get("deployment_mode") == expected_deployment_mode
            and metadata.get("fixed_initial_association") is True
            and metadata.get("mobility_lm") == "noop"
            and metadata.get("initial_attach_stagger_s") == 0.25
            and metadata.get("monitored_dl_start_s") == 6.0
            and metadata.get("monitored_ul_start_s") == 6.5
            and metadata.get("mobility_update_ms") == 200.0
            and metadata.get("ue_mobility_contract") == expected_ue_mobility_contract
            and metadata.get("ue_placement") == expected_ue_placement
            and metadata.get("claim_scope", {}).get("backhaul_switching")
            == "integration-only"
            and metadata.get("claim_scope", {}).get("monitored_ue_qos")
            == "integration-gate-only"
            and metadata.get("claim_scope", {}).get("handover_success")
            == "not-applicable"
            and metadata.get("claim_scope", {}).get("ue_pingpong")
            == "not-applicable",
            protocol_version=metadata.get("protocol_version"),
            evidence_scope=metadata.get("evidence_scope"),
            rng_seed=metadata.get("rng_seed"),
            expected_deployment_mode=expected_deployment_mode,
            observed_deployment_mode=metadata.get("deployment_mode"),
            fixed_initial_association=metadata.get("fixed_initial_association"),
            mobility_lm=metadata.get("mobility_lm"),
            initial_attach_stagger_s=metadata.get("initial_attach_stagger_s"),
            expected_ue_placement=expected_ue_placement,
            observed_ue_placement=metadata.get("ue_placement"),
            expected_ue_mobility_contract=expected_ue_mobility_contract,
            observed_ue_mobility_contract=metadata.get("ue_mobility_contract"),
            observed_mobility_update_ms=metadata.get("mobility_update_ms"),
            claim_scope=metadata.get("claim_scope"),
        )
    )
    expected_sim_time = 36.0
    gates.append(
        gate(
            "frozen_scope_and_duration",
            scope == "integration"
            and sim_time == expected_sim_time
            and policy in {"rule", "predictor"}
            and metadata.get("drain_time_s") == 2.0
            and degradation_start == 14.0
            and degradation_stop == 22.0
            and metadata.get("degradation_penalty_db") == 35.0
            and metadata.get("degradation_ramp_s") == 2.0
            and metadata.get("donor_unavailable_start_s") == -1.0
            and metadata.get("donor_unavailable_stop_s") == -1.0,
            scope=scope,
            policy=policy,
            expected_sim_time_s=expected_sim_time,
            observed_sim_time_s=sim_time,
            expected_drain_time_s=2.0,
            observed_drain_time_s=metadata.get("drain_time_s"),
            expected_degradation={"start_s": 14.0, "stop_s": 22.0, "penalty_db": 35.0, "ramp_s": 2.0},
            observed_degradation={
                "start_s": degradation_start,
                "stop_s": degradation_stop,
                "penalty_db": metadata.get("degradation_penalty_db"),
                "ramp_s": metadata.get("degradation_ramp_s"),
            },
            expected_donor_unavailable={"start_s": -1.0, "stop_s": -1.0},
            observed_donor_unavailable={
                "start_s": metadata.get("donor_unavailable_start_s"),
                "stop_s": metadata.get("donor_unavailable_stop_s"),
            },
        )
    )
    matched_arguments = read_argv0(Path(metadata["matched_config_path"]))
    policy_arguments = read_argv0(Path(metadata["policy_config_path"]))
    matched_options, matched_duplicates = option_map(matched_arguments)
    policy_options, policy_duplicates = option_map(policy_arguments)
    expected_matched_options = {
        "--sim-time": "36",
        "--stop-tail": "2",
        "--RngSeed": "1",
        "--RngRun": "6",
        "--num-uess1": "50",
        "--num-ground-ues": "50",
        "--num-tn-gnbs": "4",
        "--num-ntn-gnbs": "3",
        "--max-ues-tn": "10",
        "--max-ues-ntn": "30",
        "--use-oran": "1",
        "--lm-query-interval": "2",
        "--e2-send-interval": "2",
        "--handover-algorithm": "ns3::NrNoOpHandoverAlgorithm",
        "--rlc-mode": "um",
        "--monitored-traffic": "udp",
        "--monitored-dl-rate-mbps": "0.2",
        "--monitored-ul-rate-mbps": "0.05",
        "--monitored-packet-size": "1000",
        "--enable-harq-retx": "1",
        "--enable-fading": "1",
        "--channel-update-ms": "100",
        "--channel-condition-update-ms": "200",
        "--capacity-stratified-ue-placement": "1",
        "--ue-anchor-cluster-radius-m": "25",
        "--uav-speed-mps": "0",
        "--mobility-update-ms": "200",
        "--xhaul-shadowing-stream": "7301",
        "--xhaul-fading-stream": "7302",
        "--xhaul-degradation-start": "14",
        "--xhaul-degradation-stop": "22",
        "--xhaul-degradation-penalty-db": "35",
        "--xhaul-degradation-ramp-s": "2",
        "--xhaul-donor-unavailable-start": "-1",
        "--xhaul-donor-unavailable-stop": "-1",
        "--uav-outage-predictor-threshold": "0.5",
        "--uav-outage-predictor-consecutive": "2",
        "--uav-outage-predictor-min-dwell-s": "6",
        "--uav-outage-predictor-max-feature-gap-s": "3",
    }
    expected_policy_arguments = {
        "rule": [
            "--deployment-mode=tn-uav-satellite",
            "--enable-sat-backhaul-monitor=1",
            "--enable-uav-switching-xapp=1",
            "--enable-ai-uav-switching-xapp=0",
            "--uav-outage-predictor-mode=off",
            "--xhaul-switch-to-sat-ttt-s=5",
            "--xhaul-switch-to-tn-ttt-s=5",
            "--backhaul-aware-handover=1",
        ],
        "predictor": [
            "--deployment-mode=tn-uav-satellite",
            "--enable-sat-backhaul-monitor=1",
            "--enable-uav-switching-xapp=1",
            "--enable-ai-uav-switching-xapp=0",
            "--uav-outage-predictor-mode=active",
            f"--uav-outage-predictor-model={model_path}",
            f"--uav-outage-predictor-model-sha256={MODEL_SHA256}",
            "--xhaul-switch-to-sat-ttt-s=5",
            "--xhaul-switch-to-tn-ttt-s=5",
            "--backhaul-aware-handover=1",
        ],
    }
    mismatched_matched_options = {
        name: {"expected": expected, "observed": matched_options.get(name)}
        for name, expected in expected_matched_options.items()
        if matched_options.get(name) != expected
    }
    gates.append(
        gate(
            "frozen_cli_and_policy_contract",
            not matched_duplicates
            and not policy_duplicates
            and not mismatched_matched_options
            and policy_arguments == expected_policy_arguments[policy],
            matched_argument_count=len(matched_arguments),
            matched_duplicate_options=matched_duplicates,
            mismatched_matched_options=mismatched_matched_options,
            policy=policy,
            observed_policy_arguments=policy_arguments,
            expected_policy_arguments=expected_policy_arguments[policy],
            policy_duplicate_options=policy_duplicates,
            parsed_policy_options=policy_options,
        )
    )
    gates.append(
        gate(
            "route_contract_identity_and_pass",
            metadata.get("route_contract_identity") == "route-contract-v21"
            and metadata.get("route_contract_passed") is True
            and metadata.get("route_contract_all_checks_passed") is True,
            expected_identity="route-contract-v21",
            observed_identity=metadata.get("route_contract_identity"),
            route_contract_passed=metadata.get("route_contract_passed"),
            all_checks_passed=metadata.get("route_contract_all_checks_passed"),
        )
    )
    expected_service_thresholds = {
        "dl_min_rate_mbps": 0.18,
        "ul_min_rate_mbps": 0.045,
        "max_delay_s": 0.30,
        "min_pdr": 0.90,
    }
    observed_service_thresholds = metadata.get("service_thresholds")
    gates.append(
        gate(
            "frozen_service_thresholds",
            observed_service_thresholds == expected_service_thresholds,
            expected=expected_service_thresholds,
            observed=observed_service_thresholds,
        )
    )
    artifacts_ok, artifact_details = verify_artifact_manifest(meta_dir)
    gates.append(gate("raw_artifact_hashes", artifacts_ok, **artifact_details))

    if not missing:
        terminal = {
            "xhaul": max_csv_time(paths["xhaul"]),
            "ai": max_csv_time(paths["ai"]),
            "qos": max_csv_time(paths["qos"]),
            "satellite": (
                max_csv_time(paths["satellite"])
                if paths["satellite"].is_file() and paths["satellite"].stat().st_size > 0
                else None
            ),
            "ues1_position": max_text_time(paths["ues1_position"]),
            "ues2_position": max_text_time(paths["ues2_position"]),
            "uav_position": max_text_time(paths["uav_position"]),
            "packets": max_csv_time(paths["packets"]),
        }
        minimums = {
            "xhaul": sim_time - 2.5,
            "ai": sim_time - 2.5,
            "qos": sim_time,
            "satellite": sim_time,
            "ues1_position": sim_time,
            "ues2_position": sim_time,
            "uav_position": sim_time,
            "packets": sim_time - 0.5,
        }
        applicable_terminal_names = set(terminal)
        if policy == "no-satellite":
            applicable_terminal_names.remove("satellite")
        terminal_ok = all(
            terminal[name] is not None and terminal[name] >= minimums[name]
            for name in applicable_terminal_names
        )
        gates.append(
            gate(
                "terminal_trace_horizons",
                terminal_ok,
                observed=terminal,
                minimum=minimums,
                not_applicable=([] if policy != "no-satellite" else ["satellite"]),
            )
        )
        if is_smoke:
            onnx_details = {
                "applicable": False,
                "reason": "pre-impairment rule-arm smoke test",
            }
            gates.append(
                gate(
                    "onnx_row_probability_parity",
                    True,
                    applicable=False,
                    reason="pre-impairment rule-arm smoke test",
                )
            )
            gates.append(
                gate(
                    "online_feature_construction",
                    True,
                    applicable=False,
                    reason="pre-impairment rule-arm smoke test",
                )
            )
        else:
            onnx_gate, onnx_details = validate_onnx_rows(
                paths["ai"], model_path, probability_tolerance, required=policy == "predictor"
            )
            gates.append(onnx_gate)
            gates.append(validate_online_feature_construction(paths["ai"], paths["xhaul"]))
        exposure = uav_exposure_and_recovery_metrics(
            paths["identity"],
            paths["rsrp"],
            paths["xhaul"],
            paths["events"],
            paths["packets"],
            degradation_start,
            degradation_stop,
        )
        exposure_fraction = exposure.get("uav_served_tx_attempt_fraction")
        association_fraction = exposure.get("fresh_serving_measurement_fraction")
        expected_monitored_ues = int(metadata["monitored_ues"])
        expected_uav_ues = int(
            metadata.get("frozen_design", {}).get("expected_monitored_uav_ues", 10)
        )
        attachment_and_initial_association_passed = (
            exposure["monitored_ue_count"] == expected_monitored_ues
            and exposure["monitored_ues_with_packet_offers"] == expected_monitored_ues
            and exposure["attached_before_first_packet_tx_count"] == expected_monitored_ues
            and exposure["first_serving_measurement_count"] == expected_monitored_ues
            and exposure["initial_uav_served_monitored_ues"] == expected_uav_ues
        )
        gates.append(
            gate(
                "monitored_attachment_and_uav_exposure",
                attachment_and_initial_association_passed
                and exposure["pre_event_offered_ue_count"] == expected_monitored_ues
                and exposure["pre_event_received_ue_count"] == expected_monitored_ues
                and all(
                    exposure["pre_event_offered_ue_count_by_direction"][direction]
                    == expected_monitored_ues
                    and exposure["pre_event_received_ue_count_by_direction"][direction]
                    == expected_monitored_ues
                    for direction in ("DL", "UL")
                )
                and (
                    is_smoke
                    or (
                        exposure["monitored_ues_with_fresh_serving_measurement"]
                        == expected_monitored_ues
                        and isinstance(association_fraction, (int, float))
                        and association_fraction >= 0.80
                        and isinstance(exposure_fraction, (int, float))
                        and exposure_fraction >= 0.05
                        and exposure["distinct_uav_served_monitored_ues"]
                        == expected_uav_ues
                    )
                ),
                expected_monitored_ues=expected_monitored_ues,
                expected_initial_uav_served_monitored_ues=expected_uav_ues,
                degradation_exposure_applicable=not is_smoke,
                **exposure,
            )
        )
        pre_event_health = pre_event_xhaul_route_health(
            paths["xhaul"], paths["events"], degradation_start, sim_time
        )
        gates.append(
            gate(
                "healthy_pre_event_xhaul_and_actual_route",
                bool(pre_event_health["passed"]),
                **{key: value for key, value in pre_event_health.items() if key != "passed"},
            )
        )
        packet_metrics = packet_cohort_metrics(
            paths["packets"],
            int(metadata["monitored_ues"]),
            sim_time,
            degradation_start,
            degradation_stop,
        )
        pre_event_direction_metrics = {
            direction: packet_metrics["windows"]["pre_degradation"][direction]
            for direction in ("DL", "UL")
        }
        access_radio_baseline_passed = all(
            values["tx_attempts"] > 0
            and values["pdr"] is not None
            and values["pdr"] >= 0.90
            and values["mean_delay_s"] is not None
            and values["mean_delay_s"] <= 0.30
            and exposure["pre_event_received_ue_count_by_direction"][direction]
            == expected_monitored_ues
            for direction, values in pre_event_direction_metrics.items()
        )
        gates.append(
            gate(
                "access_radio_baseline",
                access_radio_baseline_passed,
                window="[10 s, degradation_start_s)",
                required_pdr=0.90,
                maximum_mean_delay_s=0.30,
                required_distinct_received_ues_per_direction=expected_monitored_ues,
                direction_metrics=pre_event_direction_metrics,
                distinct_received_ues_by_direction=exposure[
                    "pre_event_received_ue_count_by_direction"
                ],
            )
        )
        switching = switching_metrics(
            paths["events"], paths["xhaul"], degradation_start, degradation_stop, policy
        )
        handover = handover_metrics(
            paths["handover"], paths["handover_failure"], pingpong_window_s
        )
        handover.update(
            {
                "applicable": False,
                "reported_handover_success": "NA",
                "reported_ue_pingpong": "NA",
                "not_applicable_reason": (
                    "fixed initial association with the RSRP mobility logic disabled"
                ),
            }
        )
        coverage = service_coverage_metrics(
            paths["rsrp"],
            paths["events"],
            paths["xhaul"],
            int(metadata["expected_ues"]),
            -110.0,
            sim_time,
            degradation_start,
            degradation_stop,
        )

        gates.append(
            gate(
                "packet_event_integrity",
                bool(packet_metrics["packet_integrity"]["passed"]),
                packet_integrity=packet_metrics["packet_integrity"],
            )
        )
        gates.append(
            gate(
                "aligned_window_partition",
                bool(packet_metrics["window_partition"]["passed"]),
                window_partition=packet_metrics["window_partition"],
            )
        )
        gates.append(
            gate(
                "handover_event_integrity",
                bool(handover["trace_integrity"]["passed"]),
                handover_trace_integrity=handover["trace_integrity"],
            )
        )
        gates.append(
            gate(
                "fixed_association_has_no_handover_events",
                handover["start_events"] == 0
                and handover["end_ok_events"] == 0
                and handover["failure_trace_records"] == 0,
                start_events=handover["start_events"],
                end_ok_events=handover["end_ok_events"],
                failure_trace_records=handover["failure_trace_records"],
                handover_success="NA",
                ue_pingpong="NA",
                reason=handover["not_applicable_reason"],
            )
        )

        denominators: dict[str, int] = {}
        applicable_windows = (
            {"full", "pre_degradation"}
            if is_smoke
            else {"full", "pre_degradation", "degradation", "post_degradation"}
        )
        for window_name, direction_metrics in packet_metrics["windows"].items():
            if window_name not in applicable_windows:
                continue
            for direction, values in direction_metrics.items():
                denominators[f"packet.{window_name}.{direction}.tx_attempts"] = int(
                    values["pdr_denominator_tx_attempts"]
                )
        for window_name, values in packet_metrics["service_satisfaction"]["windows"].items():
            if window_name not in applicable_windows:
                continue
            denominators[f"service.{window_name}.ue_samples"] = int(
                values["denominator_ue_samples"]
            )
        for window_name, values in coverage["windows"].items():
            if window_name not in applicable_windows:
                continue
            denominators[f"coverage.{window_name}.ue_seconds"] = int(
                values["denominator_ue_seconds"]
            )
        if not is_smoke:
            denominators["exposure.offered_tx_attempts"] = int(
                exposure["offered_tx_attempts"]
            )
        nonpositive_denominators = {
            name: value for name, value in denominators.items() if value <= 0
        }
        gates.append(
            gate(
                "positive_applicable_metric_denominators",
                not nonpositive_denominators,
                denominators=denominators,
                nonpositive=nonpositive_denominators,
                handover_metrics="NA",
                reason=(
                    "fixed initial association; handover success and UE ping-pong are NA"
                ),
                excluded_windows=(
                    ["degradation", "post_degradation"] if is_smoke else []
                ),
            )
        )

        expected_uav_cells = set(EXPECTED_UAV_CELL_IDS)
        linked_satellite_cell_ids = set(switching["linked_satellite_policy_cell_ids"])
        satellite_recovery_complete = (
            switching["linked_satellite_policy_event_count"] == len(expected_uav_cells)
            and linked_satellite_cell_ids == expected_uav_cells
            and switching["linked_events_without_sustained_raw_recovery"] == 0
            and switching["linked_events_without_tn_policy_recovery"] == 0
        )
        raw_recovery_complete = (
            set(switching["raw_outage_cell_ids"]) == expected_uav_cells
            and set(switching["raw_recovery_cell_ids"]) == expected_uav_cells
            and set(switching["tn_recovery_route_transition_cell_ids"])
            == expected_uav_cells
        )
        gates.append(
            gate(
                "raw_outage_and_recovery_observed",
                True
                if is_smoke
                else raw_recovery_complete
                and (satellite_recovery_complete if policy != "no-satellite" else True),
                applicable=not is_smoke,
                not_applicable_reason=(
                    "pre-impairment smoke test" if is_smoke else None
                ),
                raw_outage_cells=switching["raw_outage_cells"],
                raw_outage_cell_ids=switching["raw_outage_cell_ids"],
                raw_recovery_cells=switching["raw_recovery_cells"],
                raw_recovery_cell_ids=switching["raw_recovery_cell_ids"],
                tn_recovery_route_transition_cells=switching[
                    "tn_recovery_route_transition_cells"
                ],
                tn_recovery_route_transition_cell_ids=switching[
                    "tn_recovery_route_transition_cell_ids"
                ],
                expected_uav_cell_ids=list(EXPECTED_UAV_CELL_IDS),
                linked_events_without_sustained_raw_recovery=switching[
                    "linked_events_without_sustained_raw_recovery"
                ],
                linked_events_without_tn_policy_recovery=switching[
                    "linked_events_without_tn_policy_recovery"
                ],
                recovery_definition=(
                    "first pair of consecutive 2 s RawXhaulState=HEALTHY observations "
                    "after raw outage"
                ),
            )
        )

        event_rows = read_csv_rows(paths["events"])
        changed_policy_events = [
            row
            for row in event_rows
            if row.get("EventType", "POLICY_APPLY") == "POLICY_APPLY"
            and flag(row.get("ModeChanged"))
        ]
        satellite_policy_events = [
            row
            for row in changed_policy_events
            if row.get("AppliedMode") == "SATELLITE_FALLBACK"
        ]
        ai_rows = read_csv_rows(paths["ai"])
        raw_positive_inferences = sum(
            flag(row.get("FeatureValid"))
            and flag(row.get("InferenceOk"))
            and flag(row.get("RawPrediction"))
            for row in ai_rows
        )
        controller_exercised = True
        expected_controller: str | None = None
        if policy == "predictor":
            expected_controller = "predictive-active"
            controller_exercised = raw_positive_inferences > 0 and any(
                row.get("ControllerMode") == expected_controller
                for row in satellite_policy_events
            )
        elif policy == "rule":
            expected_controller = "rule"
            controller_exercised = any(
                row.get("ControllerMode") == expected_controller
                for row in satellite_policy_events
            )

        recovery_events = exposure["recovery_events"]
        affected_recovery_events = [
            row for row in recovery_events if row["affected_monitored_ues"] > 0
        ]
        missing_packet_recovery = [
            row
            for row in affected_recovery_events
            if row["first_successful_packet_rx_s"] is None
            or row["route_to_first_successful_packet_s"] is None
            or row["route_to_first_successful_packet_s"] < 0.0
        ]
        linked_satellite_cells = set(switching["linked_satellite_policy_cell_ids"])
        linked_transition_keys = {
            (int(row["uav_cell_id"]), round(float(row["apply_time_s"]), 9))
            for row in switching["linked_satellite_policy_events"]
        }
        exposed_transition_keys = {
            (int(row["uav_cell_id"]), round(float(row["route_apply_s"]), 9))
            for row in affected_recovery_events
        }
        recovered_transition_keys = {
            (int(row["uav_cell_id"]), round(float(row["route_apply_s"]), 9))
            for row in affected_recovery_events
            if row not in missing_packet_recovery
        }
        exposed_linked_transition_keys = linked_transition_keys & exposed_transition_keys
        recovered_linked_transition_keys = linked_transition_keys & recovered_transition_keys
        exposed_linked_satellite_cells = {
            cell for cell, _ in exposed_linked_transition_keys
        }
        recovered_linked_satellite_cells = {
            cell for cell, _ in recovered_linked_transition_keys
        }

        predictor_target_events = switching["linked_satellite_policy_events"]
        future_target_passed = policy != "predictor" or (
            len(predictor_target_events) == len(expected_uav_cells)
            and linked_satellite_cells == expected_uav_cells
            and switching["unlinked_satellite_policy_events"] == 0
            and all(
                event.get("link_rule")
                == "exact UNREACHABLE target at decision +2/+4/+6 s"
                and [target.get("offset_s") for target in event.get("future_raw_targets", [])]
                == [2.0, 4.0, 6.0]
                and all(
                    target.get("raw_xhaul_state") == "UNREACHABLE"
                    for target in event.get("future_raw_targets", [])
                )
                for event in predictor_target_events
            )
        )
        gates.append(
            gate(
                "linked_future_raw_target_plus_2_4_6",
                future_target_passed,
                applicable=policy == "predictor",
                expected_uav_cell_ids=list(EXPECTED_UAV_CELL_IDS),
                linked_uav_cell_ids=sorted(linked_satellite_cell_ids),
                unlinked_satellite_policy_events=switching[
                    "unlinked_satellite_policy_events"
                ],
                linked_events=predictor_target_events,
            )
        )

        if policy == "no-satellite":
            route_exercised = switching["no_backhaul_route_transition_cells"] > 0
            policy_exercised = route_exercised
            required_route = "NO_BACKHAUL_AVAILABLE"
        else:
            route_exercised = (
                set(switching["satellite_route_transition_cell_ids"])
                == expected_uav_cells
            )
            policy_exercised = (
                len(satellite_policy_events) == len(expected_uav_cells)
                and controller_exercised
                and linked_satellite_cell_ids == expected_uav_cells
                and switching["unlinked_satellite_policy_events"] == 0
            )
            if policy in {"rule", "predictor"}:
                route_exercised = (
                    route_exercised
                    and exposed_linked_satellite_cells == expected_uav_cells
                    and recovered_linked_satellite_cells == expected_uav_cells
                )
            required_route = "SATELLITE_FALLBACK"
        gates.append(
            gate(
                "policy_exercised_with_actual_route_transition",
                True if is_smoke else policy_exercised and route_exercised,
                applicable=not is_smoke,
                not_applicable_reason=(
                    "pre-impairment smoke test" if is_smoke else None
                ),
                policy=policy,
                expected_controller=expected_controller,
                controller_exercised=controller_exercised,
                raw_positive_inferences=raw_positive_inferences,
                changed_satellite_policy_events=len(satellite_policy_events),
                required_actual_route=required_route,
                satellite_route_transition_cells=switching[
                    "satellite_route_transition_cells"
                ],
                satellite_route_transition_cell_ids=switching[
                    "satellite_route_transition_cell_ids"
                ],
                no_backhaul_route_transition_cells=switching[
                    "no_backhaul_route_transition_cells"
                ],
                linked_satellite_policy_cells=switching[
                    "linked_satellite_policy_cells"
                ],
                unlinked_satellite_policy_events=switching[
                    "unlinked_satellite_policy_events"
                ],
                exposed_linked_satellite_cells=sorted(exposed_linked_satellite_cells),
                recovered_linked_satellite_cells=sorted(recovered_linked_satellite_cells),
            )
        )

        recovery_applicable = not is_smoke and policy in {"rule", "predictor"}
        recovery_passed = (
            linked_satellite_cell_ids == expected_uav_cells
            and exposed_linked_satellite_cells == expected_uav_cells
            and recovered_linked_satellite_cells == expected_uav_cells
            and not missing_packet_recovery
            if recovery_applicable
            else True
        )
        gates.append(
            gate(
                "packet_recovery_after_satellite_apply",
                recovery_passed,
                applicable=recovery_applicable,
                route_transition_events=len(recovery_events),
                events_with_affected_monitored_ues=len(affected_recovery_events),
                missing_or_invalid_recovery_events=missing_packet_recovery[:20],
                linked_satellite_cells=sorted(linked_satellite_cell_ids),
                exposed_linked_satellite_cells=sorted(exposed_linked_satellite_cells),
                recovered_linked_satellite_cells=sorted(recovered_linked_satellite_cells),
                expected_uav_cell_ids=list(EXPECTED_UAV_CELL_IDS),
            )
        )

        metrics = {
            "switching": switching,
            "handover": handover,
            "qos": qos_metrics(paths["qos"], degradation_start, degradation_stop),
            "packet_cohort_qos": packet_metrics,
            "service_satisfaction": packet_metrics["service_satisfaction"],
            "uav_exposure_and_recovery": exposure,
            "service_effective_coverage": coverage,
            "onnx_replay": onnx_details,
        }
    else:
        metrics = {}

    return {
        "metadata_dir": str(meta_dir.resolve()),
        "metadata": metadata,
        "paths": {key: str(value) for key, value in paths.items()},
        "gates": gates,
        "gates_passed": all(item["passed"] for item in gates),
        "metrics": metrics,
    }


def projected_rows(path: Path, columns: tuple[str, ...]) -> list[tuple[str, ...]]:
    return [tuple(row.get(column, "") for column in columns) for row in read_csv_rows(path)]


def compare_projection(
    name: str,
    left_path: Path,
    right_path: Path,
    columns: tuple[str, ...],
) -> dict[str, Any]:
    left = projected_rows(left_path, columns)
    right = projected_rows(right_path, columns)
    mismatches: list[dict[str, Any]] = []
    for index, (left_row, right_row) in enumerate(zip(left, right)):
        if left_row != right_row:
            mismatches.append({"row": index, "reference": left_row, "candidate": right_row})
            if len(mismatches) == 10:
                break
    if len(left) != len(right):
        mismatches.append({"reason": "row_count", "reference": len(left), "candidate": len(right)})
    return gate(
        name,
        not mismatches,
        columns=columns,
        reference_rows=len(left),
        candidate_rows=len(right),
        first_mismatches=mismatches,
    )


def compare_offered_packet_attempts(left_path: Path, right_path: Path) -> dict[str, Any]:
    def attempts(path: Path) -> list[tuple[str, ...]]:
        return [
            tuple(row.get(column, "") for column in PACKET_OFFER_COLUMNS)
            for row in read_csv_rows(path)
            if row.get("Event") == "TX"
        ]

    left = attempts(left_path)
    right = attempts(right_path)
    mismatches: list[dict[str, Any]] = []
    for index, (left_row, right_row) in enumerate(zip(left, right)):
        if left_row != right_row:
            mismatches.append({"row": index, "reference": left_row, "candidate": right_row})
            if len(mismatches) == 10:
                break
    if len(left) != len(right):
        mismatches.append({"reason": "row_count", "reference": len(left), "candidate": len(right)})
    return gate(
        "matched_application_tx_attempts",
        not mismatches,
        columns=PACKET_OFFER_COLUMNS,
        reference_rows=len(left),
        candidate_rows=len(right),
        first_mismatches=mismatches,
    )


def nested_value(payload: dict[str, Any], path: tuple[str, ...]) -> Any:
    value: Any = payload
    for key in path:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def numeric_delta(candidate: Any, reference: Any) -> float | None:
    if isinstance(candidate, (int, float)) and isinstance(reference, (int, float)):
        return candidate - reference
    return None


def compare_runs(reference: dict[str, Any], candidate: dict[str, Any]) -> dict[str, Any]:
    ref_meta = reference["metadata"]
    cand_meta = candidate["metadata"]
    ref_paths = {key: Path(value) for key, value in reference["paths"].items()}
    cand_paths = {key: Path(value) for key, value in candidate["paths"].items()}
    gates: list[dict[str, Any]] = []
    same_design = {
        "rng_seed": ref_meta["rng_seed"] == cand_meta["rng_seed"],
        "rng_run": ref_meta["rng_run"] == cand_meta["rng_run"],
        "matched_config_sha256": (
            ref_meta["matched_config_sha256"] == cand_meta["matched_config_sha256"]
        ),
        "source_sha256": ref_meta["source_sha256"] == cand_meta["source_sha256"],
        "binary_sha256": ref_meta["binary_sha256"] == cand_meta["binary_sha256"],
        "model_sha256": ref_meta["model_sha256"] == cand_meta["model_sha256"],
        "schema_sha256": ref_meta.get("schema_sha256") == cand_meta.get("schema_sha256"),
        "radio_contract": ref_meta.get("radio_contract") == cand_meta.get("radio_contract"),
    }
    for hash_field in MATCHED_PROVENANCE_HASH_FIELDS:
        same_design[hash_field] = (
            bool(ref_meta.get(hash_field))
            and ref_meta.get(hash_field) == cand_meta.get(hash_field)
        )
    gates.append(gate("matched_design_identity", all(same_design.values()), **same_design))

    mobility_hashes: dict[str, dict[str, str]] = {}
    mobility_ok = True
    for key in ("ues1_position", "ues2_position", "uav_position"):
        ref_hash = sha256(ref_paths[key])
        cand_hash = sha256(cand_paths[key])
        mobility_hashes[key] = {"reference": ref_hash, "candidate": cand_hash}
        mobility_ok &= ref_hash == cand_hash
    gates.append(gate("matched_mobility_hashes", mobility_ok, hashes=mobility_hashes))
    gates.append(
        compare_projection(
            "matched_raw_xhaul_physical_inputs",
            ref_paths["xhaul"],
            cand_paths["xhaul"],
            PHYSICAL_XHAUL_COLUMNS,
        )
    )
    satellite_comparison_applicable = (
        ref_meta.get("policy") != "no-satellite"
        and cand_meta.get("policy") != "no-satellite"
    )
    if satellite_comparison_applicable:
        gates.append(
            compare_projection(
                "matched_satellite_snr_inputs",
                ref_paths["satellite"],
                cand_paths["satellite"],
                SATELLITE_COLUMNS,
            )
        )
    else:
        gates.append(
            gate(
                "matched_satellite_snr_inputs",
                True,
                applicable=False,
                reason="no-satellite arm intentionally disables the satellite monitor",
                reference_policy=ref_meta.get("policy"),
                candidate_policy=cand_meta.get("policy"),
            )
        )
    gates.append(compare_offered_packet_attempts(ref_paths["packets"], cand_paths["packets"]))
    linked_episode_comparison_applicable = (
        ref_meta.get("policy") != "no-satellite"
        and cand_meta.get("policy") != "no-satellite"
    )
    reference_episode_ids = nested_value(
        reference["metrics"], ("switching", "linked_outage_episode_ids")
    )
    candidate_episode_ids = nested_value(
        candidate["metrics"], ("switching", "linked_outage_episode_ids")
    )
    gates.append(
        gate(
            "matched_linked_outage_episode_ids",
            (
                bool(reference_episode_ids)
                and reference_episode_ids == candidate_episode_ids
                if linked_episode_comparison_applicable
                else True
            ),
            applicable=linked_episode_comparison_applicable,
            reference=reference_episode_ids,
            candidate=candidate_episode_ids,
        )
    )

    metric_paths = {
        "pdr_full_all": ("packet_cohort_qos", "windows", "full", "ALL", "pdr"),
        "pdr_degradation_all": (
            "packet_cohort_qos",
            "windows",
            "degradation",
            "ALL",
            "pdr",
        ),
        "pdr_degradation_dl": (
            "packet_cohort_qos",
            "windows",
            "degradation",
            "DL",
            "pdr",
        ),
        "pdr_degradation_ul": (
            "packet_cohort_qos",
            "windows",
            "degradation",
            "UL",
            "pdr",
        ),
        "coverage_degradation": (
            "service_effective_coverage",
            "windows",
            "degradation",
            "rate",
        ),
        "service_satisfaction_degradation": (
            "service_satisfaction",
            "windows",
            "degradation",
            "rate",
        ),
        "switch_mode_changed_events": ("switching", "mode_changed_events"),
        "switch_delay_from_raw_s": (
            "switching",
            "mean_satellite_apply_minus_raw_unreachable_s",
        ),
        "route_interruption_s": (
            "switching",
            "mean_route_loss_to_satellite_restore_s",
        ),
    }
    paired_metrics: dict[str, Any] = {}
    for name, path in metric_paths.items():
        ref_value = nested_value(reference["metrics"], path)
        cand_value = nested_value(candidate["metrics"], path)
        paired_metrics[name] = {
            "reference": ref_value,
            "candidate": cand_value,
            "candidate_minus_reference": numeric_delta(cand_value, ref_value),
        }
    return {
        "rng_seed": ref_meta["rng_seed"],
        "rng_run": ref_meta["rng_run"],
        "reference_policy": ref_meta["policy"],
        "candidate_policy": cand_meta["policy"],
        "gates": gates,
        "gates_passed": all(item["passed"] for item in gates),
        "paired_metrics": paired_metrics,
    }


def write_metric_csv(path: Path, runs: list[dict[str, Any]]) -> None:
    fields = [
        "rng_seed",
        "rng_run",
        "policy",
        "gates_passed",
        "wall_clock_seconds",
        "handover_starts",
        "handover_matched_successes",
        "handover_success_rate",
        "pingpong_events",
        "pingpong_denominator",
        "pingpong_rate",
        "full_tx_packets",
        "full_rx_packets",
        "full_pdr",
        "degradation_tx_packets",
        "degradation_rx_packets",
        "degradation_pdr",
        "degradation_coverage_numerator",
        "degradation_coverage_denominator",
        "degradation_coverage_rate",
        "degradation_service_numerator",
        "degradation_service_denominator",
        "degradation_service_rate",
        "mode_changed_events",
        "switch_delay_from_raw_s",
        "route_interruption_s",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for run in runs:
            metadata = run["metadata"]
            status = read_json(Path(run["metadata_dir"]) / "process-status.json")
            metrics = run.get("metrics", {})
            handover = metrics.get("handover", {})
            full = nested_value(
                metrics, ("packet_cohort_qos", "windows", "full", "ALL")
            ) or {}
            degradation = nested_value(
                metrics, ("packet_cohort_qos", "windows", "degradation", "ALL")
            ) or {}
            coverage = nested_value(
                metrics, ("service_effective_coverage", "windows", "degradation")
            ) or {}
            switching = metrics.get("switching", {})
            service = nested_value(
                metrics, ("service_satisfaction", "windows", "degradation")
            ) or {}
            writer.writerow(
                {
                    "rng_seed": metadata["rng_seed"],
                    "rng_run": metadata["rng_run"],
                    "policy": metadata["policy"],
                    "gates_passed": int(run["gates_passed"]),
                    "wall_clock_seconds": status.get("wall_clock_seconds"),
                    "handover_starts": handover.get("start_events", 0),
                    "handover_matched_successes": handover.get("matched_start_end_ok", 0),
                    "handover_success_rate": "NA",
                    "pingpong_events": "NA",
                    "pingpong_denominator": "NA",
                    "pingpong_rate": "NA",
                    "full_tx_packets": full.get("tx_attempts"),
                    "full_rx_packets": full.get("rx_packets"),
                    "full_pdr": full.get("pdr"),
                    "degradation_tx_packets": degradation.get("tx_attempts"),
                    "degradation_rx_packets": degradation.get("rx_packets"),
                    "degradation_pdr": degradation.get("pdr"),
                    "degradation_coverage_numerator": coverage.get("covered_ue_seconds"),
                    "degradation_coverage_denominator": coverage.get("denominator_ue_seconds"),
                    "degradation_coverage_rate": coverage.get("rate"),
                    "degradation_service_numerator": service.get("passing_ue_samples"),
                    "degradation_service_denominator": service.get("denominator_ue_samples"),
                    "degradation_service_rate": service.get("rate"),
                    "mode_changed_events": switching.get("mode_changed_events"),
                    "switch_delay_from_raw_s": switching.get(
                        "mean_satellite_apply_minus_raw_unreachable_s"
                    ),
                    "route_interruption_s": switching.get(
                        "mean_route_loss_to_satellite_restore_s"
                    ),
                }
            )


def write_pair_csv(path: Path, comparisons: list[dict[str, Any]]) -> None:
    fields = [
        "rng_seed",
        "rng_run",
        "reference_policy",
        "candidate_policy",
        "gates_passed",
        "metric",
        "reference",
        "candidate",
        "candidate_minus_reference",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for comparison in comparisons:
            for metric, values in comparison["paired_metrics"].items():
                writer.writerow(
                    {
                        "rng_seed": comparison["rng_seed"],
                        "rng_run": comparison["rng_run"],
                        "reference_policy": comparison["reference_policy"],
                        "candidate_policy": comparison["candidate_policy"],
                        "gates_passed": int(comparison["gates_passed"]),
                        "metric": metric,
                        **values,
                    }
                )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--metadata-dir",
        action="append",
        required=True,
        type=Path,
        help="integration-arm metadata directory; supply exactly rule and predictor",
    )
    parser.add_argument("--output", required=True, type=Path, help="validation JSON output")
    parser.add_argument("--probability-tolerance", type=float, default=1e-5)
    parser.add_argument("--pingpong-window-s", type=float, default=10.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    runs: list[dict[str, Any]] = []
    errors: list[dict[str, str]] = []
    for meta_dir in args.metadata_dir:
        try:
            runs.append(
                validate_run(meta_dir.resolve(), args.probability_tolerance, args.pingpong_window_s)
            )
        except Exception as error:  # preserve a machine-readable failure report
            errors.append({"metadata_dir": str(meta_dir.resolve()), "error": repr(error)})

    grouped: dict[tuple[int, int], list[dict[str, Any]]] = defaultdict(list)
    for run in runs:
        metadata = run["metadata"]
        grouped[(int(metadata["rng_seed"]), int(metadata["rng_run"]))].append(run)
    comparisons: list[dict[str, Any]] = []
    design_gates: list[dict[str, Any]] = []
    for group in grouped.values():
        scopes = {str(run["metadata"].get("scope")) for run in group}
        policies = {str(run["metadata"].get("policy")) for run in group}
        if scopes == {"integration"}:
            expected = {"rule", "predictor"}
        else:
            expected = set()
        design_gates.append(
            gate(
                "complete_policy_group",
                policies == expected and len(group) == len(expected),
                rng_seed=group[0]["metadata"]["rng_seed"],
                rng_run=group[0]["metadata"]["rng_run"],
                scopes=sorted(scopes),
                observed_policies=sorted(policies),
                expected_policies=sorted(expected),
            )
        )
        if len(group) < 2:
            continue
        reference = next((run for run in group if run["metadata"]["policy"] == "rule"), group[0])
        for candidate in group:
            if candidate is not reference:
                try:
                    comparisons.append(compare_runs(reference, candidate))
                except Exception as error:
                    errors.append(
                        {
                            "metadata_dir": candidate["metadata_dir"],
                            "error": f"paired comparison failed: {error!r}",
                        }
                    )

    design_gates.append(
        gate(
            "one_seed_integration_only",
            len(runs) == 2
            and len(grouped) == 1
            and all(run["metadata"].get("rng_seed") == 1 for run in runs)
            and all(run["metadata"].get("rng_run") == 6 for run in runs)
            and all(
                run["metadata"].get("evidence_status")
                == "integration-only; not manuscript efficacy evidence"
                for run in runs
            ),
            run_count=len(runs),
            group_count=len(grouped),
            allowed_inference="integration and wall-time gate only; no efficacy or generalization claim",
        )
    )
    overall = (
        len(runs) == 2
        and not errors
        and all(run["gates_passed"] for run in runs)
        and all(item["passed"] for item in design_gates)
        and len(comparisons) == 1
        and all(comparison["gates_passed"] for comparison in comparisons)
    )
    payload = {
        "protocol_version": "uav-reviewer-integration-validator-v5",
        "evidence_status": "one-seed integration-only; not manuscript efficacy evidence",
        "overall_passed": overall,
        "run_count": len(runs),
        "comparison_count": len(comparisons),
        "design_gates": design_gates,
        "errors": errors,
        "runs": runs,
        "comparisons": comparisons,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    metrics_csv = args.output.with_name(f"{args.output.stem}-network-metrics.csv")
    pairs_csv = args.output.with_name(f"{args.output.stem}-paired-comparisons.csv")
    write_metric_csv(metrics_csv, runs)
    write_pair_csv(pairs_csv, comparisons)
    print(f"validation={'PASS' if overall else 'FAIL'} json={args.output}")
    print(f"network_metrics={metrics_csv}")
    print(f"paired_comparisons={pairs_csv}")
    return 0 if overall else 1


if __name__ == "__main__":
    sys.exit(main())
