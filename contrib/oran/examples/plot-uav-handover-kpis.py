#!/usr/bin/env python3
"""Summarize handover success, failure, and ping-pong KPIs."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


DEFAULT_DATASET_ROOT = Path("results/nr/tn-ntn/ai-dataset-v2")
DEFAULT_AI_RUN = Path(
    "results/nr/tn-ntn/"
    "tn-uav-satellite_ueS1_50_ueS2_50_tnGnb_4_ntnGnb_3_tnCap_20_ntnCap_10_hyst_2_ai-switching-sat"
)


def seed_from_name(name: str) -> int:
    match = re.search(r"seed(\d+)", name)
    return int(match.group(1)) if match else -1


def parse_ho_trace(path: Path) -> pd.DataFrame:
    rows = []
    if not path.exists() or path.stat().st_size == 0:
        return pd.DataFrame(columns=["Time", "IMSI", "TargetCell", "Type"])
    for line in path.read_text().splitlines():
        parts = line.split()
        if not parts:
            continue
        row = {"Time": float(parts[0])}
        for token in parts[1:]:
            if "=" not in token:
                continue
            key, value = token.split("=", 1)
            if key in {"IMSI", "TargetCell"}:
                row[key] = int(value)
            elif key == "Type":
                row[key] = value
        if {"IMSI", "TargetCell", "Type"}.issubset(row):
            rows.append(row)
    return pd.DataFrame(rows)


def count_ping_pong(success: pd.DataFrame, window_s: float) -> int:
    if success.empty:
        return 0
    count = 0
    for _, group in success.sort_values("Time").groupby("IMSI"):
        prev_prev_cell = None
        prev_time = None
        prev_cell = None
        for row in group.itertuples(index=False):
            current_cell = int(row.TargetCell)
            current_time = float(row.Time)
            if (
                prev_prev_cell is not None
                and prev_time is not None
                and current_cell == prev_prev_cell
                and current_time - prev_time <= window_s
            ):
                count += 1
            prev_prev_cell = prev_cell
            prev_cell = current_cell
            prev_time = current_time
    return count


def count_ric_ho_failures(log_file: Path) -> dict[str, int]:
    counts: dict[str, int] = {}
    if not log_file.exists():
        return counts
    for line in log_file.read_text(errors="ignore").splitlines():
        match = re.search(r"\b(HO_FAIL_[A-Z0-9_]+)", line)
        if match:
            reason = match.group(1)
            counts[reason] = counts.get(reason, 0) + 1
    return counts


def summarize_run(run_dir: Path, scenario: str, seed: int, pingpong_window_s: float) -> dict:
    success = parse_ho_trace(run_dir / "handover-trace.tr")
    failure_file = run_dir / "handover-failure-trace.tr"
    failure_count = 0
    if failure_file.exists() and failure_file.stat().st_size > 0:
        failure_count = len(failure_file.read_text().splitlines())

    tn_success = int(success["Type"].eq("TN").sum()) if not success.empty else 0
    ntn_success = int(success["Type"].eq("NTN").sum()) if not success.empty else 0
    ric_failures = count_ric_ho_failures(run_dir / "ns3-oran-lm.log")
    ric_failure_count = sum(ric_failures.values())
    return {
        "Scenario": scenario,
        "Seed": seed,
        "RunDir": str(run_dir),
        "Success": int(len(success)),
        "RrcFailure": int(failure_count),
        "RicCandidateFailure": int(ric_failure_count),
        "RicLowRsrpFailure": int(ric_failures.get("HO_FAIL_LOW_RSRP", 0)),
        "TnTargetSuccess": tn_success,
        "NtnTargetSuccess": ntn_success,
        "PingPong": count_ping_pong(success, pingpong_window_s),
    }


def collect_runs(dataset_root: Path, ai_run: Path, pingpong_window_s: float) -> pd.DataFrame:
    rows = []
    for run_dir in sorted(dataset_root.glob("*-no-sat")):
        rows.append(summarize_run(run_dir, "TN + UAV, no satellite", seed_from_name(run_dir.name), pingpong_window_s))
    for run_dir in sorted(dataset_root.glob("*-sat")):
        if run_dir.name.endswith("-no-sat"):
            continue
        rows.append(summarize_run(run_dir, "TN + UAV + satellite", seed_from_name(run_dir.name), pingpong_window_s))
    if ai_run.exists():
        rows.append(summarize_run(ai_run, "TN + UAV + satellite, RF-AI", -1, pingpong_window_s))
    return pd.DataFrame(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--ai-run", type=Path, default=DEFAULT_AI_RUN)
    parser.add_argument("--pingpong-window-s", type=float, default=10.0)
    parser.add_argument("--output", type=Path, default=Path("docs/figures/uav_handover_kpis.pdf"))
    parser.add_argument("--png", type=Path, default=Path("docs/figures/uav_handover_kpis.png"))
    parser.add_argument("--csv", type=Path, default=Path("docs/figures/uav_handover_kpis.csv"))
    args = parser.parse_args()

    style_path = Path("./latex_style.mplstyle")
    if not style_path.exists():
        style_path = Path(__file__).resolve().parents[3] / "latex_style.mplstyle"
    if style_path.exists():
        plt.style.use(style_path)

    df = collect_runs(args.dataset_root, args.ai_run, args.pingpong_window_s)
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(args.csv, index=False)

    summary = (
        df.groupby("Scenario", as_index=False)
        .agg(
            SuccessMean=("Success", "mean"),
            RrcFailureMean=("RrcFailure", "mean"),
            RicCandidateFailureMean=("RicCandidateFailure", "mean"),
            RicLowRsrpFailureMean=("RicLowRsrpFailure", "mean"),
            PingPongMean=("PingPong", "mean"),
            TnSuccessMean=("TnTargetSuccess", "mean"),
            NtnSuccessMean=("NtnTargetSuccess", "mean"),
            Runs=("Scenario", "size"),
        )
    )

    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "font.size": 13,
            "axes.labelsize": 14,
            "xtick.labelsize": 11,
            "ytick.labelsize": 11,
            "legend.fontsize": 10,
        }
    )

    labels = summary["Scenario"].tolist()
    x = range(len(labels))
    fig, axes = plt.subplots(1, 2, figsize=(10.2, 4.2), constrained_layout=True)

    axes[0].bar(x, summary["TnSuccessMean"], label="TN target", color="#4C78A8")
    axes[0].bar(x, summary["NtnSuccessMean"], bottom=summary["TnSuccessMean"], label="UAV/NTN target", color="#F28E2B")
    axes[0].set_ylabel("Successful handovers")
    axes[0].set_xticks(list(x))
    axes[0].set_xticklabels(labels, rotation=16, ha="right")
    axes[0].grid(True, axis="y", linestyle="--", alpha=0.38)
    axes[0].legend(frameon=True)

    width = 0.34
    axes[1].bar(
        [i - width for i in x],
        summary["RrcFailureMean"],
        width=width,
        label="RRC HO failures",
        color="#E15759",
    )
    axes[1].bar(
        x,
        summary["RicCandidateFailureMean"],
        width=width,
        label="RIC candidate rejects",
        color="#B07AA1",
    )
    axes[1].bar(
        [i + width for i in x],
        summary["PingPongMean"],
        width=width,
        label=f"Ping-pong <= {args.pingpong_window_s:.0f}s",
        color="#59A14F",
    )
    axes[1].set_ylabel("Count")
    axes[1].set_xticks(list(x))
    axes[1].set_xticklabels(labels, rotation=16, ha="right")
    axes[1].grid(True, axis="y", linestyle="--", alpha=0.38)
    axes[1].legend(frameon=True)

    fig.savefig(args.output)
    fig.savefig(args.png, dpi=300)
    print(f"[saved] {args.output}")
    print(f"[saved] {args.png}")
    print(f"[saved] {args.csv}")
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
