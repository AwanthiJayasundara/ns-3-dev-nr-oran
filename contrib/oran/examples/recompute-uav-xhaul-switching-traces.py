#!/usr/bin/env python3
"""Recompute UAV xHaul switching traces from recorded donor distances.

This helper is for fast what-if analysis when the simulation already recorded
UAV-to-TN donor distances but the donor-link policy parameters changed. It
does not rerun ns-3 and it does not change QoS, handover, or packet-routing
traces. It only recalculates xhaul-autonomy-trace.csv fields that are derived
from the donor-backhaul RSRP and switching policy.
"""

from __future__ import annotations

import argparse
import csv
import math
import shutil
from pathlib import Path


TRACE_NAME = "xhaul-autonomy-trace.csv"


def estimate_rsrp_dbm(
    tx_power_dbm: float,
    frequency_hz: float,
    distance_m: float,
    exponent: float,
    reference_distance_m: float,
    channel_variation_db: float,
) -> float:
    d = max(distance_m, 1.0)
    d0 = max(reference_distance_m, 1.0)
    f_ghz = frequency_hz / 1e9
    l0_db = 32.4 + 20.0 * math.log10(f_ghz) + 20.0 * math.log10(d0)
    path_loss_db = l0_db if d <= d0 else l0_db + 10.0 * exponent * math.log10(d / d0)
    return tx_power_dbm - path_loss_db + channel_variation_db


def fmt(value: float) -> str:
    if abs(value + 999.0) < 1e-9:
        return "-999"
    return f"{value:.6g}"


def infer_exponent_from_label(label: str, default_exponent: float) -> float:
    if "-fspl-" in label:
        return 2.0
    if "-logdist-" in label or "-logdist-fading-" in label:
        return default_exponent
    return default_exponent


def recompute_trace(src_trace: Path, dst_trace: Path, args: argparse.Namespace) -> dict[str, int]:
    with src_trace.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fieldnames = reader.fieldnames or []

    counts: dict[str, int] = {}
    raw_state_by_cell: dict[str, str] = {}
    effective_state_by_cell: dict[str, str] = {}
    raw_since_by_cell: dict[str, float] = {}
    pathloss_exponent = (
        infer_exponent_from_label(src_trace.parent.name, args.pathloss_exponent)
        if args.infer_loss_model_from_label
        else args.pathloss_exponent
    )

    for row in rows:
        time_s = float(row["Time"])
        cell_id = row["UavCellId"]
        deployment = row["DeploymentMode"]
        distance_m = float(row["BestDonorDistanceM"])
        channel_variation_db = float(row.get("XhaulChannelVariationDb", "0") or 0.0)
        best_donor_cell = row.get("BestDonorCellId", "0")
        xhaul_connected = best_donor_cell != "0" and distance_m >= 0.0

        if xhaul_connected:
            rsrp_dbm = estimate_rsrp_dbm(
                args.tx_power_dbm,
                args.frequency_hz,
                distance_m,
                pathloss_exponent,
                args.reference_distance_m,
                channel_variation_db,
            )
            raw_state = "HEALTHY" if rsrp_dbm >= args.healthy_rsrp_dbm else "UNREACHABLE"
        else:
            rsrp_dbm = -999.0
            raw_state = "UNREACHABLE"

        if cell_id not in raw_state_by_cell:
            raw_state_by_cell[cell_id] = raw_state
            effective_state_by_cell[cell_id] = raw_state
            raw_since_by_cell[cell_id] = time_s
        elif raw_state_by_cell[cell_id] != raw_state:
            raw_state_by_cell[cell_id] = raw_state
            raw_since_by_cell[cell_id] = time_s

        age_s = time_s - raw_since_by_cell[cell_id]
        effective_state = effective_state_by_cell[cell_id]
        if raw_state != effective_state:
            needed_ttt = args.switch_to_tn_ttt_s if raw_state == "HEALTHY" else args.switch_to_sat_ttt_s
            if age_s >= needed_ttt:
                effective_state_by_cell[cell_id] = raw_state
                effective_state = raw_state

        sat_healthy = row.get("SatBackhaulHealthy", "0") == "1"
        switching_available = deployment != "tn-only"
        switching_active = switching_available and effective_state != "HEALTHY"
        satellite_allowed = deployment == "tn-uav-satellite"
        use_satellite = switching_active and satellite_allowed and sat_healthy
        direct_tn = effective_state == "HEALTHY"

        if use_satellite:
            backhaul_mode = "SATELLITE_FALLBACK"
            ric_state = "NEAR_RT_RIC_SWITCHING_TO_SATELLITE_BACKHAUL"
            uav_mode = "NEAR_RT_RIC_EMERGENCY_SWITCHING_WITH_SATELLITE_BACKHAUL"
            normal_ho = "1"
        elif direct_tn:
            backhaul_mode = "TN_DIRECT"
            ric_state = "TN_E2"
            uav_mode = "TN_CONTROLLED_COVERAGE_EXTENSION"
            normal_ho = "1"
        else:
            backhaul_mode = "NO_BACKHAUL_AVAILABLE"
            ric_state = "NEAR_RT_RIC_SWITCHING_NO_SATELLITE_BACKHAUL" if switching_active else "LOCAL_AUTONOMY"
            uav_mode = "AUTONOMOUS_LOCAL_ISLAND"
            normal_ho = "0"

        row["XhaulConnected"] = "1" if xhaul_connected else "0"
        row["XhaulRsrpDbm"] = fmt(rsrp_dbm)
        row["RawXhaulState"] = raw_state
        row["RawXhaulStateAgeSec"] = fmt(age_s)
        row["XhaulState"] = effective_state
        row["DonorUnavailableActive"] = "0"
        row["UavSwitchingXappAvailable"] = "1" if switching_available else "0"
        row["UavSwitchingXappState"] = "ACTIVE" if switching_active else ("STANDBY" if switching_available else "DISABLED")
        row["BackhaulMode"] = backhaul_mode
        row["RicControlState"] = ric_state
        row["ActiveUavRic"] = "TN_NEAR_RT_RIC" if switching_available else "LOCAL_UAV_AUTONOMY"
        row["NormalUeHandoverAllowed"] = normal_ho
        row["UavMode"] = uav_mode
        counts[backhaul_mode] = counts.get(backhaul_mode, 0) + 1

    with dst_trace.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    return counts


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-dir", default="results/nr/tn-ntn/ai-dataset-v1")
    parser.add_argument("--output-dir", default="results/nr/tn-ntn/ai-dataset-v1-recomputed-gamma5p6")
    parser.add_argument("--tx-power-dbm", type=float, default=46.0)
    parser.add_argument("--frequency-hz", type=float, default=4.0e9)
    parser.add_argument("--pathloss-exponent", type=float, default=5.6)
    parser.add_argument("--reference-distance-m", type=float, default=100.0)
    parser.add_argument("--healthy-rsrp-dbm", type=float, default=-100.0)
    parser.add_argument("--switch-to-sat-ttt-s", type=float, default=5.0)
    parser.add_argument("--switch-to-tn-ttt-s", type=float, default=5.0)
    parser.add_argument(
        "--infer-loss-model-from-label",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use gamma=2.0 for folders containing -fspl- and --pathloss-exponent for logdist folders.",
    )
    parser.add_argument("--replace-output", action="store_true")
    args = parser.parse_args()

    src_root = Path(args.input_dir)
    dst_root = Path(args.output_dir)
    if not src_root.exists():
        raise SystemExit(f"Input directory not found: {src_root}")
    if dst_root.exists():
        if not args.replace_output:
            raise SystemExit(f"Output exists: {dst_root}. Use --replace-output to overwrite it.")
        shutil.rmtree(dst_root)

    shutil.copytree(src_root, dst_root)

    all_counts: dict[str, int] = {}
    traces = sorted(dst_root.glob(f"*/{TRACE_NAME}"))
    for trace in traces:
        counts = recompute_trace(src_root / trace.parent.name / TRACE_NAME, trace, args)
        for key, value in counts.items():
            all_counts[key] = all_counts.get(key, 0) + value
        print(f"{trace.parent.name}: {counts}")

    readme = dst_root / "README_DERIVED.txt"
    readme.write_text(
        "Derived dataset generated by recompute-uav-xhaul-switching-traces.py.\n"
        "Only xhaul-autonomy-trace.csv was recomputed from recorded donor distances.\n"
        "QoS, packet routing, handover, and ns-3 logs were copied from the source run and were not rerun.\n"
        f"tx_power_dbm={args.tx_power_dbm}\n"
        f"frequency_hz={args.frequency_hz}\n"
        f"pathloss_exponent={args.pathloss_exponent}\n"
        f"reference_distance_m={args.reference_distance_m}\n"
        f"healthy_rsrp_dbm={args.healthy_rsrp_dbm}\n"
        f"switch_to_sat_ttt_s={args.switch_to_sat_ttt_s}\n"
        f"switch_to_tn_ttt_s={args.switch_to_tn_ttt_s}\n"
        f"aggregate_backhaul_counts={all_counts}\n",
        encoding="utf-8",
    )
    print(f"Aggregate: {all_counts}")
    print(f"Wrote derived dataset: {dst_root}")


if __name__ == "__main__":
    main()
