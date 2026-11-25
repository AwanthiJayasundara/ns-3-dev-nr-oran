#!/usr/bin/env python3
"""Validate the 100-UE Gate-2 O-RAN RSRP mobility integration run.

This validator deliberately reports event numerators and denominators from raw
records.  It does not turn one diagnostic seed into a manuscript result.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import sqlite3
import sys
from collections import Counter, defaultdict
from contextlib import closing
from pathlib import Path
from typing import Any


EXPECTED_PROTOCOL = "uav-reviewer-mobility-gate2-v2"
EXPECTED_MOVER_IDS = {0, 10, 20, 30}
EXPECTED_UAV_CELLS = {5, 6, 7}
REQUIRED_RAW_TRACES = {
    "mobility-manifest-applied.csv", "ue-identity-map.csv", "rsrp-trace.tr",
    "handover-trace.tr", "handover-failure-trace.tr", "monitored-packet-events.csv",
    "ml-ho-dataset.csv", "ues1-position-trace.tr", "ues2-position-trace.tr",
    "uav-position-trace.tr", "xhaul-autonomy-trace.csv", "uav-switching-events.csv",
    "final-flow-report.txt",
}
POSITION_TOLERANCE_M = 0.01
TRACE_TIME_TOLERANCE_S = 1e-4
LOCAL_LATITUDE_DEG = 53.3498
LOCAL_LONGITUDE_DEG = -6.2603
METERS_PER_LATITUDE_DEG = 111320.0
METERS_PER_LONGITUDE_DEG = METERS_PER_LATITUDE_DEG * math.cos(
    math.radians(LOCAL_LATITUDE_DEG)
)
EXPECTED_MANIFEST_COLUMNS = [
    "ue_index",
    "cohort",
    "source_cell",
    "target_cell",
    "start_east_m",
    "start_north_m",
    "end_east_m",
    "end_north_m",
    "start_time_s",
    "stop_time_s",
    "altitude_m",
]
ALLOCATOR_ERROR = re.compile(
    r"AddressSanitizer|heap-use-after-free|double free|free\(\):|invalid next size|"
    r"corrupted size|memory corruption|Segmentation fault",
    re.IGNORECASE,
)
KV_RE = re.compile(r"([A-Za-z][A-Za-z0-9_]*)=([^\s]+)")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--metadata-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def load_manifest(path: Path) -> list[dict[str, Any]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames != EXPECTED_MANIFEST_COLUMNS:
            raise ValueError(f"unexpected manifest header in {path}: {reader.fieldnames!r}")
        rows: list[dict[str, Any]] = []
        for raw in reader:
            rows.append(
                {
                    "ue_index": int(raw["ue_index"]),
                    "cohort": raw["cohort"],
                    "source_cell": raw["source_cell"],
                    "target_cell": raw["target_cell"],
                    "start_east_m": float(raw["start_east_m"]),
                    "start_north_m": float(raw["start_north_m"]),
                    "end_east_m": float(raw["end_east_m"]),
                    "end_north_m": float(raw["end_north_m"]),
                    "start_time_s": float(raw["start_time_s"]),
                    "stop_time_s": float(raw["stop_time_s"]),
                    "altitude_m": float(raw["altitude_m"]),
                }
            )
    return rows


def same_manifest(left: list[dict[str, Any]], right: list[dict[str, Any]]) -> bool:
    if len(left) != len(right):
        return False
    for a, b in zip(sorted(left, key=lambda row: row["ue_index"]),
                    sorted(right, key=lambda row: row["ue_index"])):
        if set(a) != set(b):
            return False
        for key in a:
            if isinstance(a[key], float):
                if not math.isclose(a[key], b[key], rel_tol=0.0, abs_tol=1e-9):
                    return False
            elif a[key] != b[key]:
                return False
    return True


def parse_identity(path: Path) -> tuple[dict[int, int], dict[int, tuple[str, int]]]:
    ue_to_imsi: dict[int, int] = {}
    imsi_to_ue: dict[int, tuple[str, int]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        required = {"Group", "UE", "IMSI", "Ipv4Address"}
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError(f"bad identity-map header: {reader.fieldnames!r}")
        for row in reader:
            group = row["Group"]
            ue = int(row["UE"])
            imsi = int(row["IMSI"])
            if group not in {"S1", "S2"} or not 0 <= ue < 50 or imsi <= 0:
                raise ValueError(f"unexpected identity row: {row!r}")
            if group == "S1":
                if ue in ue_to_imsi:
                    raise ValueError(f"duplicate S1 UE index {ue}")
                ue_to_imsi[ue] = imsi
            if imsi in imsi_to_ue:
                raise ValueError(f"duplicate IMSI {imsi}")
            imsi_to_ue[imsi] = (group, ue)
    return ue_to_imsi, imsi_to_ue


def load_e2_node_ids(path: Path) -> dict[int, int]:
    # RIC candidate rows identify E2 nodes. NR handover traces identify IMSIs.
    with closing(sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True)) as db:
        rows = db.execute("SELECT imsi, nodeid FROM nrue").fetchall()
    return dict(rows)


def parse_rsrp(path: Path) -> tuple[dict[int, tuple[float, int]], float]:
    first_serving: dict[int, tuple[float, int]] = {}
    max_time = -math.inf
    with path.open(encoding="utf-8") as handle:
        header = handle.readline().split()
        expected = ["Time", "IMSI", "RNTI", "CellId", "CellType", "RSRP", "RSRQ", "Serving", "CCID"]
        if header != expected:
            raise ValueError(f"bad RSRP header: {header!r}")
        for line in handle:
            fields = line.split()
            if not fields:
                continue
            if len(fields) != 9:
                raise ValueError(f"bad RSRP trace row: {line!r}")
            time_s = float(fields[0])
            if not math.isfinite(time_s) or time_s < 0:
                raise ValueError(f"bad RSRP trace time: {line!r}")
            max_time = max(max_time, time_s)
            if int(fields[7]) != 1:
                continue
            imsi = int(fields[1])
            cell = int(fields[3])
            previous = first_serving.get(imsi)
            if previous is None or time_s < previous[0]:
                first_serving[imsi] = (time_s, cell)
    return first_serving, max_time


def position_coordinates(latitude: float, longitude: float, altitude: float) -> tuple[float, ...]:
    return (
        (longitude - LOCAL_LONGITUDE_DEG) * METERS_PER_LONGITUDE_DEG,
        (latitude - LOCAL_LATITUDE_DEG) * METERS_PER_LATITUDE_DEG,
        altitude,
    )


def validate_positions(
    path: Path, prefix: str, node_count: int, first_epoch: int,
    manifest_by_node: dict[int, dict[str, Any]],
) -> dict[str, Any]:
    expected_epochs = set(range(first_epoch, 32))
    expected_keys = {(node, epoch) for node in range(node_count) for epoch in expected_epochs}
    positions: dict[tuple[int, float], tuple[float, ...]] = {}
    duplicate_keys: list[tuple[int, float]] = []
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            fields = line.split()
            if not fields:
                continue
            if len(fields) != 5 or not re.fullmatch(re.escape(prefix) + r"[0-9]+", fields[1]):
                raise ValueError(f"bad position row at {path}:{line_number}: {line!r}")
            timestamp = float(fields[0])
            node = int(fields[1].removeprefix(prefix))
            geo = tuple(map(float, fields[2:]))
            if not all(math.isfinite(value) for value in (timestamp, *geo)):
                raise ValueError(f"nonfinite position row at {path}:{line_number}")
            key = (node, timestamp)
            if key in positions:
                duplicate_keys.append(key)
            positions[key] = position_coordinates(*geo)

    missing = expected_keys - positions.keys()
    unexpected = positions.keys() - expected_keys
    errors: list[dict[str, Any]] = []
    max_mover_error = 0.0
    max_stationary_displacement = 0.0
    for (node, epoch), observed in sorted(positions.items()):
        if (node, epoch) in unexpected:
            continue
        corridor = manifest_by_node.get(node)
        if corridor is None:
            expected = positions.get((node, first_epoch))
            if expected is None:
                continue
            error = math.dist(observed, expected)
            max_stationary_displacement = max(max_stationary_displacement, error)
        else:
            start = corridor["start_time_s"]
            stop = corridor["stop_time_s"]
            fraction = min(1.0, max(0.0, (epoch - start) / (stop - start)))
            expected = (
                corridor["start_east_m"] + fraction * (corridor["end_east_m"] - corridor["start_east_m"]),
                corridor["start_north_m"] + fraction * (corridor["end_north_m"] - corridor["start_north_m"]),
                corridor["altitude_m"],
            )
            error = math.dist(observed, expected)
            max_mover_error = max(max_mover_error, error)
        if error > POSITION_TOLERANCE_M:
            errors.append({"node": node, "time_s": epoch, "position_error_m": error,
                           "expected_east_north_alt_m": expected,
                           "observed_east_north_alt_m": observed})
    return {
        "passed": not (missing or unexpected or duplicate_keys or errors),
        "expected_node_epochs": len(expected_keys),
        "observed_unique_node_epochs": len(positions),
        "last_sample_s": max((time for _, time in positions), default=None),
        "position_tolerance_m": POSITION_TOLERANCE_M,
        "max_mover_error_m": max_mover_error if manifest_by_node else None,
        "max_stationary_displacement_m": max_stationary_displacement,
        "missing_node_epochs": len(missing),
        "unexpected_node_epochs": len(unexpected),
        "duplicate_node_epochs": len(duplicate_keys),
        "position_error_count": len(errors),
        "first_missing_node_epochs": sorted(missing)[:20],
        "first_unexpected_node_epochs": sorted(unexpected)[:20],
        "first_position_errors": errors[:20],
    }


def validate_fixed_backhaul(xhaul_path: Path, event_path: Path) -> dict[str, Any]:
    # RIC starts at 2.5 s, queries every two seconds, and stops when traffic ends at 30 s.
    expected_keys = {(cell, 4.5 + 2 * epoch) for cell in EXPECTED_UAV_CELLS for epoch in range(13)}
    required_values = {
        "XhaulConnected": "1",
        "RawXhaulState": "HEALTHY",
        "XhaulState": "HEALTHY",
        "BackhaulMode": "TN_DIRECT",
        "ActualRouteMode": "TN_DIRECT",
        "XhaulDegradationActive": "0",
        "DonorUnavailableActive": "0",
        "UavSwitchingXappAvailable": "0",
    }
    samples: Counter[tuple[int, float]] = Counter()
    bad_states: list[dict[str, Any]] = []
    with xhaul_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        required_fields = set(required_values) | {"UavCellId", "Time"}
        if not required_fields.issubset(reader.fieldnames or []):
            raise ValueError(f"bad xhaul CSV header: {reader.fieldnames!r}")
        for row in reader:
            cell, time_s = int(row["UavCellId"]), float(row["Time"])
            if not math.isfinite(time_s):
                raise ValueError(f"nonfinite xhaul timestamp: {row!r}")
            samples[(cell, time_s)] += 1
            mismatches = {name: row[name] for name, value in required_values.items() if row[name] != value}
            if mismatches:
                bad_states.append({"uav_cell_id": cell, "time_s": time_s, "mismatches": mismatches})
    with event_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if not {"ApplyTime", "UavCellId", "EventType", "RouteChanged"}.issubset(reader.fieldnames or []):
            raise ValueError(f"bad switching-event CSV header: {reader.fieldnames!r}")
        events = list(reader)
    missing = expected_keys - samples.keys()
    unexpected = samples.keys() - expected_keys
    duplicates = sum(count - 1 for count in samples.values())
    return {
        "passed": not (missing or unexpected or duplicates or bad_states or events),
        "expected_samples": len(expected_keys),
        "observed_samples": sum(samples.values()),
        "missing_samples": sorted(missing),
        "unexpected_samples": sorted(unexpected),
        "duplicate_samples": duplicates,
        "non_healthy_or_non_tn_direct_samples": len(bad_states),
        "first_bad_state_samples": bad_states[:20],
        "switching_event_records": len(events),
        "first_switching_event_records": events[:20],
    }


def parse_event_line(line: str) -> dict[str, Any]:
    timestamp = float(line.split(maxsplit=1)[0])
    if not math.isfinite(timestamp) or not 0 <= timestamp <= 32.0:
        raise ValueError(f"invalid handover time: {line!r}")
    fields: dict[str, Any] = {"time": timestamp}
    for key, value in KV_RE.findall(line):
        if key in fields:
            raise ValueError(f"duplicate handover field {key}: {line!r}")
        fields[key] = value
    return fields


def parse_handover_events(
    handover_path: Path, failure_path: Path
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    starts: list[dict[str, Any]] = []
    successes: list[dict[str, Any]] = []
    failures: list[dict[str, Any]] = []
    for line in handover_path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        event = parse_event_line(line)
        if event.get("Event") == "START":
            starts.append(event)
        elif event.get("Event") == "END_OK":
            successes.append(event)
        else:
            raise ValueError(f"unexpected handover event: {line!r}")
    for line in failure_path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            event = parse_event_line(line)
            if "Reason" not in event or event.get("Side") not in {"GNB", "UE"}:
                raise ValueError(f"unexpected handover failure event: {line!r}")
            failures.append(event)
    return starts, successes, failures


def parse_candidate_eligibility(
    path: Path,
    opportunities: list[dict[str, Any]],
    ue_to_imsi: dict[int, int],
    e2_by_imsi: dict[int, int],
) -> dict[int, dict[str, Any]]:
    by_e2_target: dict[tuple[int, int], dict[float, dict[str, Any]]] = defaultdict(dict)
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        required = {
            "time", "ueId", "servingCell", "servingRsrp", "candidateCell",
            "candidateRsrp", "candidateEffLoad", "candidateCap",
        }
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError(f"bad decision CSV header: {reader.fieldnames!r}")
        for row in reader:
            e2_id = int(row["ueId"])
            candidate = int(row["candidateCell"])
            time_s = float(row["time"])
            candidate_rsrp = float(row["candidateRsrp"])
            serving_rsrp = float(row["servingRsrp"])
            load = int(row["candidateEffLoad"])
            cap = int(row["candidateCap"])
            if not all(math.isfinite(value) for value in (time_s, candidate_rsrp, serving_rsrp)):
                raise ValueError(f"nonfinite candidate measurement: {row!r}")
            qualifies = (
                candidate_rsrp > serving_rsrp + 2.0
                and candidate_rsrp >= -110.0
                and cap > 0
                and 0 <= load < cap
            )
            detail = {
                "time_s": time_s,
                "serving_cell": int(row["servingCell"]),
                "serving_rsrp_dbm": serving_rsrp,
                "candidate_rsrp_dbm": candidate_rsrp,
                "candidate_effective_load": load,
                "candidate_capacity": cap,
                "qualifies": qualifies,
            }
            epochs = by_e2_target[(e2_id, candidate)]
            if time_s in epochs:
                raise ValueError(f"duplicate RIC candidate epoch: {row!r}")
            epochs[time_s] = detail

    results: dict[int, dict[str, Any]] = {}
    for opportunity in opportunities:
        ue = opportunity["ue_index"]
        imsi = ue_to_imsi[ue]
        e2_id = e2_by_imsi[imsi]
        source = cell_label_to_id(opportunity["source_cell"])
        target = cell_label_to_id(opportunity["target_cell"])
        samples = sorted(by_e2_target.get((e2_id, target), {}).items())
        onset: float | None = None
        ttt_satisfied: float | None = None
        previous_qualifying_time: float | None = None
        qualifying_samples: list[dict[str, Any]] = []
        for time_s, detail in samples:
            if time_s < opportunity["start_time_s"]:
                continue
            if detail["serving_cell"] == source and detail["qualifies"]:
                qualifying_samples.append(detail)
                if previous_qualifying_time is not None and math.isclose(
                    time_s - previous_qualifying_time, 2.0, rel_tol=0.0, abs_tol=TRACE_TIME_TOLERANCE_S
                ):
                    onset = previous_qualifying_time
                    ttt_satisfied = time_s
                    break
                previous_qualifying_time = time_s
            else:
                previous_qualifying_time = None
        results[ue] = {
            "imsi": imsi,
            "e2_node_id": e2_id,
            "source_cell": source,
            "target_cell": target,
            "eligible_for_frozen_ttt": ttt_satisfied is not None,
            "eligibility_onset_s": onset,
            "ttt_satisfied_s": ttt_satisfied,
            "qualifying_sample_count": len(qualifying_samples),
            "first_qualifying_samples": qualifying_samples[:3],
        }
    return results


def cell_label_to_id(label: str) -> int:
    if label.startswith("TN"):
        return int(label[2:])
    if label.startswith("UAV"):
        return 4 + int(label[3:])
    raise ValueError(f"unknown cell label {label!r}")


def handover_metrics(
    starts: list[dict[str, Any]],
    successes: list[dict[str, Any]],
    failures: list[dict[str, Any]],
    identity: dict[int, tuple[str, int]],
    eligibility: dict[int, dict[str, Any]],
) -> dict[str, Any]:
    def key(event: dict[str, Any]) -> tuple[int, int, int]:
        return int(event["IMSI"]), int(event["SourceCell"]), int(event["TargetCell"])

    starts_by_id: dict[int, dict[str, Any]] = {}
    invalid_starts: list[dict[str, Any]] = []
    duplicate_starts: list[int] = []
    for event in starts:
        attempt_id = int(event["AttemptId"])
        imsi, source, target = key(event)
        if attempt_id <= 0 or imsi not in identity or source == target or not (
            1 <= source <= 7 and 1 <= target <= 7
        ):
            invalid_starts.append(event)
        elif attempt_id in starts_by_id:
            duplicate_starts.append(attempt_id)
        else:
            starts_by_id[attempt_id] = event

    reported_outcomes: dict[int, list[dict[str, Any]]] = defaultdict(list)
    unlinked_failure_callbacks: list[dict[str, Any]] = []
    for event in successes + failures:
        attempt_id = int(event["AttemptId"])
        if attempt_id == 0 and "Reason" in event and int(event["IMSI"]) in identity:
            # The UE and gNB can report the same failure. Only the first
            # callback can consume the active attempt and carry its positive ID.
            unlinked_failure_callbacks.append(event)
        else:
            reported_outcomes[attempt_id].append(event)

    duplicate_outcomes = sorted(attempt for attempt, outcomes in reported_outcomes.items() if len(outcomes) > 1)
    orphan_outcomes = sorted(reported_outcomes.keys() - starts_by_id.keys())
    mismatched_outcomes: list[dict[str, Any]] = []
    matched_successes: list[dict[str, Any]] = []
    matched_failures: list[dict[str, Any]] = []
    reconciled_ids: set[int] = set()
    for attempt_id, start in starts_by_id.items():
        outcomes = reported_outcomes.get(attempt_id, [])
        if len(outcomes) != 1:
            continue
        outcome = outcomes[0]
        stated_start = float(outcome["StartTime"])
        delay = outcome["time"] - start["time"]
        is_success = outcome.get("Event") == "END_OK"
        reported_delay = float(outcome["CompletionDelay"]) if is_success else None
        timing_matches = delay >= 0 and math.isclose(
            stated_start, start["time"], rel_tol=0.0, abs_tol=TRACE_TIME_TOLERANCE_S
        ) and (reported_delay is None or (
            reported_delay >= 0 and math.isclose(
                reported_delay, delay, rel_tol=0.0, abs_tol=TRACE_TIME_TOLERANCE_S
            )
        ))
        if key(start) != key(outcome) or not timing_matches:
            mismatched_outcomes.append({"start": start, "outcome": outcome})
            continue
        reconciled_ids.add(attempt_id)
        if is_success:
            matched_successes.append(outcome)
        else:
            matched_failures.append(outcome)
    unresolved = sorted(starts_by_id.keys() - reconciled_ids)

    by_mover_imsi = {row["imsi"]: row for row in eligibility.values()}
    registered_starts = [event for event in starts_by_id.values() if int(event["IMSI"]) in by_mover_imsi]

    def follows_opportunity(event: dict[str, Any]) -> bool:
        imsi, source, target = key(event)
        opportunity = by_mover_imsi.get(imsi, {})
        eligible_time = opportunity.get("ttt_satisfied_s")
        return (
            eligible_time is not None and event["time"] + TRACE_TIME_TOLERANCE_S >= eligible_time
            and source == opportunity["source_cell"] and target == opportunity["target_cell"]
        )

    intended_starts = [event for event in registered_starts if follows_opportunity(event)]
    intended_started_imsis = {int(event["IMSI"]) for event in intended_starts}
    intended_successes = {
        int(event["IMSI"]) for event in matched_successes
        if follows_opportunity(starts_by_id[int(event["AttemptId"])])
    }

    completed_by_imsi: dict[int, list[dict[str, Any]]] = defaultdict(list)
    for event in matched_successes:
        completed_by_imsi[int(event["IMSI"])].append(event)
    pingpong_events: list[dict[str, Any]] = []
    eligible_adjacent_pairs = 0
    discontinuous_adjacent_pairs = 0
    for imsi, events in completed_by_imsi.items():
        ordered = sorted(events, key=lambda event: (event["time"], int(event["AttemptId"])))
        for previous, current in zip(ordered, ordered[1:]):
            elapsed_s = current["time"] - previous["time"]
            if previous["TargetCell"] != current["SourceCell"]:
                discontinuous_adjacent_pairs += 1
                continue
            eligible_adjacent_pairs += 1
            if previous["SourceCell"] == current["TargetCell"] and elapsed_s <= 10.0:
                pingpong_events.append({"imsi": imsi, "first_attempt": int(previous["AttemptId"]),
                                        "second_attempt": int(current["AttemptId"]), "delta_s": elapsed_s})

    def rate(numerator: int, denominator: int) -> float | None:
        return numerator / denominator if denominator else None

    mover_pingpong = {event["imsi"] for event in pingpong_events if event["imsi"] in by_mover_imsi}
    registered_opportunities = len(eligibility)
    unique_starts = len(starts_by_id)
    completed = len(matched_successes)
    completion_delays = [float(event["CompletionDelay"]) for event in matched_successes]
    return {
        "attempt_ids_valid_and_unique": not (invalid_starts or duplicate_starts),
        "all_attempts_reconciled": not (unresolved or duplicate_outcomes or orphan_outcomes or mismatched_outcomes),
        "raw_handover_starts": len(starts),
        "raw_handover_end_ok_records": len(successes),
        "raw_handover_failure_records": len(failures),
        "registered_opportunities_denominator": registered_opportunities,
        "all_starts_numerator": unique_starts,
        "registered_mover_starts_numerator": len(registered_starts),
        "correct_source_target_attempt_starts": len(intended_starts),
        "correct_target_starts_numerator": len(intended_started_imsis),
        "initiation_rate_all_starts_over_registered_opportunities": rate(unique_starts, registered_opportunities),
        "correct_target_initiation_rate": rate(len(intended_started_imsis), registered_opportunities),
        "successful_attempts_numerator": completed,
        "attempt_success_denominator": unique_starts,
        "attempt_success_rate": rate(completed, unique_starts),
        "completed_to_intended_target_numerator": len(intended_successes),
        "opportunity_success_rate": rate(len(intended_successes), registered_opportunities),
        "failures": len(matched_failures),
        "matched_failure_reasons": dict(Counter(event["Reason"] for event in matched_failures)),
        "first_unlinked_failure_callbacks": unlinked_failure_callbacks[:20],
        "unlinked_failure_callback_records": len(unlinked_failure_callbacks),
        "invalid_start_records": invalid_starts[:20],
        "duplicate_start_attempt_ids": duplicate_starts,
        "unresolved_attempt_ids": unresolved,
        "orphan_outcome_attempt_ids": orphan_outcomes,
        "duplicate_outcome_attempt_ids": duplicate_outcomes,
        "mismatched_outcomes": mismatched_outcomes[:20],
        "completion_delay_denominator": len(completion_delays),
        "mean_completion_delay_s": sum(completion_delays) / len(completion_delays) if completion_delays else None,
        "pingpong_events_numerator": len(pingpong_events),
        "completed_handovers_denominator": completed,
        "pingpong_per_completed_handover": rate(len(pingpong_events), completed),
        "eligible_adjacent_pairs_denominator": eligible_adjacent_pairs,
        "discontinuous_adjacent_pairs": discontinuous_adjacent_pairs,
        "pingpong_per_eligible_adjacent_pair": rate(len(pingpong_events), eligible_adjacent_pairs),
        "moving_ues_with_pingpong_numerator": len(mover_pingpong),
        "registered_moving_ues_denominator": registered_opportunities,
        "moving_ues_with_pingpong_rate": rate(len(mover_pingpong), registered_opportunities),
        "pingpong_events": pingpong_events,
    }


def validate_packet_events(path: Path, traffic_end_s: float, receive_end_s: float) -> dict[str, Any]:
    tx: dict[tuple[int, str, int], tuple[float, float, int]] = {}
    rx: dict[tuple[int, str, int], tuple[float, float, int]] = {}
    duplicate_tx: list[tuple[int, str, int]] = []
    duplicate_rx: list[tuple[int, str, int]] = []
    invalid_rows: list[dict[str, str]] = []
    first_tx_s_by_ue: dict[int, float] = {}
    max_time: float | None = None
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        required = {"Time", "UE", "Dir", "Event", "PacketBytes", "Sequence", "SenderTimestampSec"}
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError(f"bad packet-event header: {reader.fieldnames!r}")
        for row in reader:
            time_s = float(row["Time"])
            event = row["Event"]
            ue, direction, sequence = int(row["UE"]), row["Dir"], int(row["Sequence"])
            sender_time, packet_bytes = float(row["SenderTimestampSec"]), int(row["PacketBytes"])
            # UDP client's TX trace is before adding its 12-byte SeqTs header.
            # The sink's RX trace includes that header and sees the 1,000-byte packet.
            expected_sizes = {"TX": 988, "RX": 1000}
            if (
                not 0 <= ue < 50 or direction not in {"DL", "UL"} or sequence < 0
                or not 0 <= sender_time <= time_s <= receive_end_s or sender_time >= traffic_end_s
                or expected_sizes.get(event) != packet_bytes
                or (event == "TX" and not math.isclose(time_s, sender_time, rel_tol=0.0, abs_tol=1e-9))
            ):
                invalid_rows.append(row)
                continue
            max_time = time_s if max_time is None else max(max_time, time_s)
            key = (ue, direction, sequence)
            value = (time_s, sender_time, packet_bytes)
            if event == "TX":
                if key in tx:
                    duplicate_tx.append(key)
                tx[key] = value
                first_tx_s_by_ue[ue] = min(time_s, first_tx_s_by_ue.get(ue, time_s))
            else:
                if key in rx:
                    duplicate_rx.append(key)
                rx[key] = value
    orphan_rx = sorted(set(rx) - set(tx))
    mismatched_rx = {
        key for key in set(tx) & set(rx)
        if not math.isclose(tx[key][1], rx[key][1], rel_tol=0.0, abs_tol=1e-9)
        or rx[key][0] + 1e-12 < tx[key][0]
    }
    counts: dict[str, dict[str, Any]] = {}
    for direction in ("DL", "UL"):
        direction_tx = {key for key in tx if key[1] == direction}
        direction_rx = {key for key in rx if key[1] == direction} - mismatched_rx
        received = len(direction_tx & direction_rx)
        counts[direction] = {
            "tx": len(direction_tx),
            "rx_matched": received,
            "pdr": received / len(direction_tx) if direction_tx else None,
            "ues_with_tx": len({key[0] for key in direction_tx}),
            "ues_with_rx": len({key[0] for key in direction_tx & direction_rx}),
        }
    return {
        "counts": counts,
        "duplicate_tx": len(duplicate_tx),
        "duplicate_rx": len(duplicate_rx),
        "orphan_rx": len(orphan_rx),
        "mismatched_rx": len(mismatched_rx),
        "invalid_rows": len(invalid_rows),
        "first_invalid_rows": invalid_rows[:20],
        "first_tx_s_by_ue": first_tx_s_by_ue,
        "max_event_time_s": max_time,
        "integrity_passed": not (duplicate_tx or duplicate_rx or orphan_rx or mismatched_rx or invalid_rows),
    }


def verify_frozen_inputs(path: Path, required_paths: set[Path] | None = None) -> dict[str, Any]:
    seen: set[Path] = set()
    duplicates: list[str] = []
    mismatches: list[dict[str, str]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        digest, filename = line.split(maxsplit=1)
        candidate = Path(filename).resolve()
        if candidate in seen:
            duplicates.append(filename)
        seen.add(candidate)
        actual = sha256(candidate) if candidate.is_file() else "MISSING"
        if actual != digest:
            mismatches.append({"path": filename, "expected": digest, "actual": actual})
    missing = [str(candidate) for candidate in sorted((required_paths or set()) - seen)]
    return {"checked": len(seen), "mismatches": mismatches, "missing_required_paths": missing,
            "duplicate_paths": duplicates, "passed": bool(seen) and not (mismatches or missing or duplicates)}


def peak_rss_kib(path: Path) -> int | None:
    match = re.search(
        r"Maximum resident set size \(kbytes\):\s*(\d+)",
        path.read_text(encoding="utf-8", errors="replace"),
    )
    return int(match.group(1)) if match else None


def main() -> int:
    args = parse_args()
    meta = args.metadata_dir.resolve()
    report: dict[str, Any] = {
        "validator": EXPECTED_PROTOCOL,
        "metadata_dir": str(meta),
        "completed_experiment": False,
        "claim_scope": "integration/crash and trace reconciliation only",
        "errors": [],
    }
    gates: dict[str, bool] = {}
    try:
        run = read_json(meta / "run.json")
        process = read_json(meta / "process-status.json")
        result_dir = Path(run["result_dir"]).resolve()
        report.update({"run_metadata": run, "process_status": process,
                       "peak_rss_kib": peak_rss_kib(meta / "time.txt")})
        gates["protocol_identity"] = (
            run.get("protocol_version") == EXPECTED_PROTOCOL
            and run.get("rng_seed") == 1 and run.get("rng_run") == 6
            and run.get("sim_time_s") == 30.0 and run.get("drain_time_s") == 2.0
        )
        gates["process_exit_zero"] = process.get("exit_code") == 0
        stderr_text = (meta / "stderr.log").read_text(encoding="utf-8", errors="replace")
        stdout_text = (meta / "stdout.log").read_text(encoding="utf-8", errors="replace")
        gates["allocator_diagnostics_clean"] = not ALLOCATOR_ERROR.search(stderr_text + "\n" + stdout_text)
        frozen = verify_frozen_inputs(meta / "frozen-inputs.sha256")
        report["frozen_inputs"] = frozen
        gates["frozen_inputs_intact"] = frozen["passed"]
        raw = verify_frozen_inputs(meta / "raw-artifacts.sha256", {result_dir / name for name in REQUIRED_RAW_TRACES})
        report["raw_artifacts"] = raw
        gates["raw_artifacts_intact"] = raw["passed"]

        registered_path = Path(run["trajectory_manifest_path"])
        applied_path = result_dir / "mobility-manifest-applied.csv"
        registered_manifest = load_manifest(registered_path)
        applied_manifest = load_manifest(applied_path)
        gates["manifest_applied_exactly"] = (
            same_manifest(registered_manifest, applied_manifest)
            and sha256(registered_path) == run["trajectory_manifest_sha256"] == sha256(applied_path)
        )
        gates["registered_opportunity_count"] = (
            len(registered_manifest) == 4
            and {row["ue_index"] for row in registered_manifest} == EXPECTED_MOVER_IDS
            and all(row["cohort"] == "S1" and 0 <= row["start_time_s"] < row["stop_time_s"] < 30
                    for row in registered_manifest)
        )
        corridors = {row["ue_index"]: row for row in registered_manifest}
        positions = {
            "s1": validate_positions(result_dir / "ues1-position-trace.tr", "UE", 50, 1, corridors),
            "s2": validate_positions(result_dir / "ues2-position-trace.tr", "UE_S2_", 50, 5, {}),
            "uav": validate_positions(result_dir / "uav-position-trace.tr", "UAV", 3, 1, {}),
        }
        report["position_validation"] = positions
        gates["scripted_paths_and_stationary_nodes"] = all(group["passed"] for group in positions.values())
        backhaul = validate_fixed_backhaul(
            result_dir / "xhaul-autonomy-trace.csv", result_dir / "uav-switching-events.csv"
        )
        report["fixed_backhaul"] = backhaul
        gates["fixed_healthy_tn_direct_backhaul"] = backhaul["passed"]

        ue_to_imsi, imsi_to_ue = parse_identity(result_dir / "ue-identity-map.csv")
        database = Path(run["database_path"])
        e2_by_imsi = load_e2_node_ids(database)
        gates["ric_identity_mapping"] = (
            set(e2_by_imsi) == set(imsi_to_ue) and len(set(e2_by_imsi.values())) == 100
        )
        report["ric_identity_mapping"] = {
            "database_path": str(database), "sha256": sha256(database),
            "e2_nodes": len(e2_by_imsi), "identity_imsis": len(imsi_to_ue),
            "missing_imsis": sorted(imsi_to_ue.keys() - e2_by_imsi.keys()),
            "unexpected_imsis": sorted(e2_by_imsi.keys() - imsi_to_ue.keys()),
        }
        first_serving, max_rsrp_time = parse_rsrp(result_dir / "rsrp-trace.tr")
        starts, successes, failures = parse_handover_events(
            result_dir / "handover-trace.tr",
            result_dir / "handover-failure-trace.tr",
        )
        packet = validate_packet_events(result_dir / "monitored-packet-events.csv", 30.0, 32.0)
        gates["identity_population"] = (
            len(ue_to_imsi) == 50
            and len(imsi_to_ue) == 100
            and set(imsi_to_ue.values()) == {(group, ue) for group in ("S1", "S2") for ue in range(50)}
        )

        expected_initial: dict[int, int] = {}
        for ue in range(50):
            if ue < 40:
                expected_initial[ue] = ue // 10 + 1
            else:
                expected_initial[ue] = 5 + (ue - 40) % 3
        s1_attachment_mismatches: list[dict[str, Any]] = []
        s1_attached_before_traffic = 0
        for ue, expected_cell in expected_initial.items():
            imsi = ue_to_imsi.get(ue)
            observed = first_serving.get(imsi) if imsi is not None else None
            first_tx = packet["first_tx_s_by_ue"].get(ue)
            if observed is not None and first_tx is not None and observed[0] <= first_tx + 1e-9:
                s1_attached_before_traffic += 1
            if observed is None or observed[1] != expected_cell:
                s1_attachment_mismatches.append(
                    {"ue": ue, "imsi": imsi, "expected_cell": expected_cell, "observed": observed}
                )
        all_attached_by_95 = sum(
            1 for imsi in imsi_to_ue if imsi in first_serving and first_serving[imsi][0] <= 9.5 + 1e-9
        )
        gates["all_100_initially_attached"] = all_attached_by_95 == 100
        gates["all_s1_attached_before_traffic"] = s1_attached_before_traffic == 50
        gates["initial_s1_anchor_mapping"] = not s1_attachment_mismatches
        s2_attachment_mismatches = [
            {"ue": ue, "imsi": imsi, "expected_cell": 5 + ue % 3, "observed": first_serving.get(imsi)}
            for imsi, (group, ue) in imsi_to_ue.items() if group == "S2"
            and (imsi not in first_serving or first_serving[imsi][1] != 5 + ue % 3)
        ]
        gates["initial_s2_anchor_mapping"] = not s2_attachment_mismatches

        eligibility = parse_candidate_eligibility(
            result_dir / "ml-ho-dataset.csv", registered_manifest, ue_to_imsi, e2_by_imsi
        )
        gates["all_four_opportunities_eligible"] = (
            len(eligibility) == 4
            and all(item["eligible_for_frozen_ttt"] for item in eligibility.values())
        )

        handover = handover_metrics(starts, successes, failures, imsi_to_ue, eligibility)
        report["handover_metrics"] = handover
        gates["at_least_one_handover_start"] = handover["all_starts_numerator"] > 0
        gates["attempt_ids_unique"] = handover["attempt_ids_valid_and_unique"]
        gates["all_attempts_reconciled"] = handover["all_attempts_reconciled"]

        gates["trace_horizons_complete"] = (
            all((group["last_sample_s"] or -1.0) >= 31.0 for group in positions.values())
            and max_rsrp_time >= 31.0
            and (packet["max_event_time_s"] or -1.0) >= 29.5
        )
        gates["packet_event_integrity"] = packet["integrity_passed"]
        gates["all_monitored_ues_transmitted"] = (
            packet["counts"]["DL"]["ues_with_tx"] == 50
            and packet["counts"]["UL"]["ues_with_tx"] == 50
        )

        report.update(
            {
                "manifest": {
                    "path": run["trajectory_manifest_path"],
                    "sha256": sha256(registered_path),
                    "registered_rows": registered_manifest,
                    "applied_path": str(applied_path),
                    "applied_exactly": gates["manifest_applied_exactly"],
                },
                "attachment": {
                    "identity_count": len(imsi_to_ue),
                    "all_attached_by_9_5_s": all_attached_by_95,
                    "s1_attached_before_first_tx": s1_attached_before_traffic,
                    "s1_anchor_mismatches": s1_attachment_mismatches,
                    "s2_anchor_mismatches": s2_attachment_mismatches,
                },
                "eligibility": eligibility,
                "packet_metrics": packet,
                "trace_horizons_s": {
                    "s1_position": positions["s1"]["last_sample_s"],
                    "s2_position": positions["s2"]["last_sample_s"],
                    "uav_position": positions["uav"]["last_sample_s"],
                    "rsrp": max_rsrp_time if math.isfinite(max_rsrp_time) else None,
                    "packet_event": packet["max_event_time_s"],
                },
            }
        )
    except Exception as error:  # preserve a machine-readable failure report
        report["errors"].append(f"{type(error).__name__}: {error}")

    report["gates"] = gates
    report["overall_passed"] = bool(gates) and all(gates.values()) and not report["errors"]
    report["completed_experiment"] = report["overall_passed"]
    report["status"] = "PASS_INTEGRATION_ONLY" if report["overall_passed"] else "FAILED_NON_RESULT"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"{report['status']} json={args.output.resolve()}")
    return 0 if report["overall_passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
