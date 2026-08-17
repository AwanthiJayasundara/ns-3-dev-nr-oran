#!/usr/bin/env python3
"""Plot static, oracle-reactive, ISAC-reactive, and RF-predictive results.

The script discovers ns-3 run directories from one or more experiment tags.
It never invents a missing method: before the RF pipeline is implemented, the
three available methods are plotted and RF is reported as missing.  Once
``isac-rf-predictive`` result directories exist, they are included
automatically.

The current ``qos-vs-time.txt`` schema contains the monitored central UEs
(UES1).  Consequently, every QoS output from this script is explicitly named
and labelled as a central-UE result.
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


METHOD_ORDER = [
    "static",
    "oracle-reactive",
    "isac-reactive",
    "isac-rf-predictive",
]
METHOD_LABELS = {
    "static": "Static UAV",
    "oracle-reactive": "Oracle reactive",
    "isac-reactive": "ISAC reactive",
    "isac-rf-predictive": "ISAC + RF predictive",
}
METHOD_COLORS = {
    "static": "#4D4D4D",
    "oracle-reactive": "#1F77B4",
    "isac-reactive": "#E67E22",
    "isac-rf-predictive": "#2CA02C",
}
METHOD_MARKERS = {
    "static": "o",
    "oracle-reactive": "s",
    "isac-reactive": "^",
    "isac-rf-predictive": "D",
}


def parse_run_name(name: str) -> dict[str, int | str] | None:
    method = next(
        (candidate for candidate in METHOD_ORDER if f"-{candidate}-clustered-" in name),
        None,
    )
    central_match = re.search(r"-(\d+)central-", name)
    outer_match = re.search(r"-(\d+)out-", name)
    seed_match = re.search(r"-seed(\d+)-", name)
    if method is None or central_match is None or outer_match is None or seed_match is None:
        return None
    central = int(central_match.group(1))
    outer = int(outer_match.group(1))
    return {
        "method": method,
        "central_ues": central,
        "outer_ues": outer,
        "total_ues": central + outer,
        "seed": int(seed_match.group(1)),
    }


def discover_runs(root: Path, tags: list[str]) -> list[tuple[Path, dict[str, int | str]]]:
    runs: list[tuple[Path, dict[str, int | str]]] = []
    for qos_path in root.rglob("qos-vs-time.txt"):
        run_dir = qos_path.parent
        if tags and not any(tag in run_dir.name for tag in tags):
            continue
        metadata = parse_run_name(run_dir.name)
        if metadata is not None:
            runs.append((run_dir, metadata))
    return sorted(
        runs,
        key=lambda item: (
            METHOD_ORDER.index(str(item[1]["method"])),
            int(item[1]["total_ues"]),
            int(item[1]["seed"]),
        ),
    )


def summarize_qos(
    run_dir: Path,
    metadata: dict[str, int | str],
    warmup_s: float,
    outage_threshold_mbps: float,
) -> dict[str, float | int | str]:
    qos = pd.read_csv(run_dir / "qos-vs-time.txt")
    required = {"Time", "UE", "Dir", "Throughput", "PDR"}
    missing = required.difference(qos.columns)
    if missing:
        raise ValueError(f"{run_dir}: missing QoS columns {sorted(missing)}")

    for column in ["Time", "UE", "Throughput", "PDR"]:
        qos[column] = pd.to_numeric(qos[column], errors="coerce")
    dl = qos[(qos["Dir"] == "DL") & (qos["Time"] >= warmup_s)].dropna(
        subset=["UE", "Throughput", "PDR"]
    )
    if dl.empty:
        raise ValueError(f"{run_dir}: no DL QoS samples at or after {warmup_s:g} s")

    per_ue = dl.groupby("UE", as_index=False).agg(
        throughput_mbps=("Throughput", "mean"),
        pdr_percent=("PDR", "mean"),
    )
    result: dict[str, float | int | str] = dict(metadata)
    result.update(
        {
            "run_directory": str(run_dir),
            "central_dl_p5_throughput_mbps": float(
                np.quantile(per_ue["throughput_mbps"], 0.05)
            ),
            "central_dl_mean_pdr_percent": float(per_ue["pdr_percent"].mean()),
            "central_dl_outage_fraction": float(
                (dl["Throughput"] <= outage_threshold_mbps).mean()
            ),
        }
    )
    return result


def confidence_summary(per_run: pd.DataFrame) -> pd.DataFrame:
    metrics = [
        "central_dl_p5_throughput_mbps",
        "central_dl_mean_pdr_percent",
        "central_dl_outage_fraction",
    ]
    rows: list[dict[str, float | int | str]] = []
    for (method, total_ues), group in per_run.groupby(["method", "total_ues"], sort=False):
        row: dict[str, float | int | str] = {
            "method": method,
            "total_ues": int(total_ues),
            "seeds": int(group["seed"].nunique()),
        }
        for metric in metrics:
            values = group[metric].to_numpy(float)
            row[f"{metric}_mean"] = float(np.mean(values))
            row[f"{metric}_ci95"] = (
                float(1.96 * np.std(values, ddof=1) / math.sqrt(len(values)))
                if len(values) > 1
                else 0.0
            )
        rows.append(row)
    return pd.DataFrame(rows)


def plot_load_comparison(summary: pd.DataFrame, output_dir: Path) -> None:
    specifications = [
        (
            "central_dl_p5_throughput_mbps",
            "Central-UE DL 5th-percentile throughput (Mbit/s)",
        ),
        ("central_dl_mean_pdr_percent", "Central-UE mean DL PDR (%)"),
        ("central_dl_outage_fraction", "Central-UE DL outage-sample fraction"),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(14.2, 4.3), constrained_layout=True)
    for method in METHOD_ORDER:
        selected = summary[summary["method"] == method].sort_values("total_ues")
        if selected.empty:
            continue
        for axis, (metric, ylabel) in zip(axes, specifications):
            axis.errorbar(
                selected["total_ues"],
                selected[f"{metric}_mean"],
                yerr=selected[f"{metric}_ci95"],
                label=METHOD_LABELS[method],
                color=METHOD_COLORS[method],
                marker=METHOD_MARKERS[method],
                linewidth=2.0,
                capsize=3,
            )
            axis.set_xlabel("Total ground UEs")
            axis.set_ylabel(ylabel)
            axis.set_xticks(sorted(summary["total_ues"].unique()))
            axis.grid(True, linestyle="--", alpha=0.35)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=max(1, len(labels)), frameon=False)
    fig.suptitle("Method comparison (QoS is for the monitored central-UE set)", y=1.07)
    for suffix in ["png", "pdf"]:
        fig.savefig(output_dir / f"isac-method-comparison-central-qos.{suffix}", dpi=300)
    plt.close(fig)


def load_time_series(run_dir: Path, warmup_s: float) -> pd.DataFrame:
    qos = pd.read_csv(run_dir / "qos-vs-time.txt")
    for column in ["Time", "Throughput", "PDR"]:
        qos[column] = pd.to_numeric(qos[column], errors="coerce")
    dl = qos[(qos["Dir"] == "DL") & (qos["Time"] >= warmup_s)]
    return dl.groupby("Time", as_index=False).agg(
        mean_throughput_mbps=("Throughput", "mean"),
        mean_pdr_percent=("PDR", "mean"),
    )


def plot_middle_load_time_series(
    runs: list[tuple[Path, dict[str, int | str]]],
    output_dir: Path,
    warmup_s: float,
    middle_load: int,
) -> None:
    frames: list[pd.DataFrame] = []
    for run_dir, metadata in runs:
        if int(metadata["total_ues"]) != middle_load:
            continue
        frame = load_time_series(run_dir, warmup_s)
        frame["method"] = str(metadata["method"])
        frame["seed"] = int(metadata["seed"])
        frames.append(frame)
    if not frames:
        return
    time_data = pd.concat(frames, ignore_index=True)
    averaged = time_data.groupby(["method", "Time"], as_index=False).agg(
        mean_throughput_mbps=("mean_throughput_mbps", "mean"),
        mean_pdr_percent=("mean_pdr_percent", "mean"),
    )

    fig, axes = plt.subplots(2, 1, figsize=(8.8, 7.0), sharex=True, constrained_layout=True)
    for method in METHOD_ORDER:
        selected = averaged[averaged["method"] == method]
        if selected.empty:
            continue
        axes[0].plot(
            selected["Time"],
            selected["mean_throughput_mbps"],
            label=METHOD_LABELS[method],
            color=METHOD_COLORS[method],
            linewidth=2.0,
        )
        axes[1].plot(
            selected["Time"],
            selected["mean_pdr_percent"],
            label=METHOD_LABELS[method],
            color=METHOD_COLORS[method],
            linewidth=2.0,
        )
    axes[0].set_ylabel("Central-UE mean DL throughput (Mbit/s)")
    axes[1].set_ylabel("Central-UE mean DL PDR (%)")
    axes[1].set_xlabel("Simulation time (s)")
    for axis in axes:
        axis.grid(True, linestyle="--", alpha=0.35)
        axis.legend(frameon=False)
    fig.suptitle(f"Central-UE QoS over time ({middle_load} total UEs)")
    for suffix in ["png", "pdf"]:
        fig.savefig(output_dir / f"isac-method-timeseries-{middle_load}ues.{suffix}", dpi=300)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--results-root",
        type=Path,
        default=Path("results/nr/tn-ntn"),
        help="Parent directory containing ns-3 run directories",
    )
    parser.add_argument(
        "--tags",
        nargs="+",
        required=True,
        help="Experiment-tag substrings to include (baseline and later RF tags may be combined)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("results/plots/isac-method-comparison"),
    )
    parser.add_argument("--warmup-s", type=float, default=20.0)
    parser.add_argument("--outage-threshold-mbps", type=float, default=0.01)
    parser.add_argument("--middle-load", type=int, default=90)
    args = parser.parse_args()

    runs = discover_runs(args.results_root, args.tags)
    if not runs:
        raise SystemExit(
            f"No matching completed runs found under {args.results_root} for tags {args.tags}"
        )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    records = [
        summarize_qos(run_dir, metadata, args.warmup_s, args.outage_threshold_mbps)
        for run_dir, metadata in runs
    ]
    per_run = pd.DataFrame(records)
    summary = confidence_summary(per_run)
    per_run.to_csv(args.output_dir / "isac-method-per-run-central-qos.csv", index=False)
    summary.to_csv(args.output_dir / "isac-method-summary-central-qos.csv", index=False)
    plot_load_comparison(summary, args.output_dir)
    plot_middle_load_time_series(runs, args.output_dir, args.warmup_s, args.middle_load)

    present = set(per_run["method"])
    missing = [METHOD_LABELS[method] for method in METHOD_ORDER if method not in present]
    print(f"Processed {len(per_run)} runs; plots written to {args.output_dir}")
    if missing:
        print("Missing methods (no curve fabricated): " + ", ".join(missing))


if __name__ == "__main__":
    main()
