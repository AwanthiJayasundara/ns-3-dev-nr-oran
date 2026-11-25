#!/usr/bin/env python3
"""Verify TN -> unavailable -> satellite -> TN packet behavior from raw events."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def number(value: str) -> float:
    return float(value)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    events = rows(args.result_dir / "uav-switching-events.csv")
    packets = rows(args.result_dir / "monitored-packet-events.csv")
    route_changes = [row for row in events if row.get("RouteChanged") == "1"]

    def first_route(mode: str, after: float) -> float:
        candidates = [
            number(row["ApplyTime"])
            for row in route_changes
            if row.get("ActualRouteMode") == mode and number(row["ApplyTime"]) >= after
        ]
        if not candidates:
            raise ValueError(f"missing route transition to {mode} after {after}")
        return min(candidates)

    route_loss = first_route("NO_BACKHAUL_AVAILABLE", 15.0)
    satellite_apply = first_route("SATELLITE_FALLBACK", route_loss)
    tn_restore = first_route("TN_DIRECT", 35.0)

    tx = [row for row in packets if row.get("Event") == "TX"]
    rx_rows = [row for row in packets if row.get("Event") == "RX"]

    def packet_key(row: dict[str, str]) -> tuple[str, str, str]:
        return (row.get("UE", ""), row.get("Dir", ""), row.get("Sequence", ""))

    tx_keys = [packet_key(row) for row in tx]
    rx_keys = [packet_key(row) for row in rx_rows]
    tx_key_set = set(tx_keys)
    rx_by_key = {packet_key(row): row for row in rx_rows}
    packet_integrity = {
        "tx_records": len(tx),
        "rx_records": len(rx_rows),
        "unique_tx_keys": len(tx_key_set),
        "unique_rx_keys": len(rx_by_key),
        "duplicate_tx_keys": len(tx_keys) - len(tx_key_set),
        "duplicate_rx_keys": len(rx_keys) - len(rx_by_key),
        "orphan_rx_keys": sum(key not in tx_key_set for key in rx_by_key),
        "invalid_tx_sequence_records": sum(int(row.get("Sequence", "-1")) < 0 for row in tx),
        "invalid_rx_sequence_records": sum(int(row.get("Sequence", "-1")) < 0 for row in rx_rows),
    }
    packet_integrity_ok = all(
        packet_integrity[name] == 0
        for name in (
            "duplicate_tx_keys",
            "duplicate_rx_keys",
            "orphan_rx_keys",
            "invalid_tx_sequence_records",
            "invalid_rx_sequence_records",
        )
    )

    def cohort(start: float, stop: float) -> dict[str, float | int | None]:
        offered = [row for row in tx if start <= number(row["Time"]) < stop]
        received = [rx_by_key[packet_key(row)] for row in offered if packet_key(row) in rx_by_key]
        delays = [
            number(row["Time"]) - number(row["SenderTimestampSec"])
            for row in received
            if number(row["SenderTimestampSec"]) >= 0.0
        ]
        return {
            "start_s": start,
            "stop_s": stop,
            "tx_attempts": len(offered),
            "rx_packets": len(received),
            "pdr": len(received) / len(offered) if offered else None,
            "mean_delay_s": sum(delays) / len(delays) if delays else None,
        }

    pre_tn = cohort(10.0, max(10.0, route_loss - 0.5))
    unavailable = cohort(route_loss + 0.1, max(route_loss + 0.1, satellite_apply - 0.3))
    satellite = cohort(satellite_apply + 0.5, max(satellite_apply + 0.5, tn_restore - 0.5))
    post_tn = cohort(tn_restore + 0.5, 50.0)

    checks = {
        "packet_trace_integrity": packet_integrity_ok,
        "ordered_route_transitions": route_loss < satellite_apply < tn_restore,
        "pre_tn_delivers": (pre_tn["pdr"] or 0.0) >= 0.8,
        "unavailable_drops": (unavailable["pdr"] or 0.0) <= 0.05,
        "satellite_delivers": (satellite["pdr"] or 0.0) >= 0.8,
        "post_tn_delivers": (post_tn["pdr"] or 0.0) >= 0.8,
        "satellite_adds_expected_delay": (
            satellite["mean_delay_s"] is not None
            and post_tn["mean_delay_s"] is not None
            and satellite["mean_delay_s"] - post_tn["mean_delay_s"] >= 0.18
        ),
    }
    payload = {
        "contract_version": "hybrid-sat-route-v1",
        "route_loss_s": route_loss,
        "satellite_apply_s": satellite_apply,
        "tn_restore_s": tn_restore,
        "cohorts": {
            "pre_tn": pre_tn,
            "unavailable": unavailable,
            "satellite": satellite,
            "post_tn": post_tn,
        },
        "checks": checks,
        "packet_integrity": packet_integrity,
        "passed": all(checks.values()),
    }
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"route_contract={'PASS' if payload['passed'] else 'FAIL'} output={args.output}")
    return 0 if payload["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
