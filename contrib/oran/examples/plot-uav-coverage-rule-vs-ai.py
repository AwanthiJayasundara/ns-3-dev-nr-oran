#!/usr/bin/env python3
"""Plot serving-link coverage for rule-based and RF-AI switching runs."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


DEFAULT_DATASET_ROOT = Path("results/nr/tn-ntn/ai-dataset-v2")
DEFAULT_AI_RUN = Path(
    "results/nr/tn-ntn/"
    "tn-uav-satellite_ueS1_50_ueS2_50_tnGnb_4_ntnGnb_3_tnCap_20_ntnCap_10_hyst_2_ai-switching-sat"
)


def load_coverage(run_dir: Path, label: str, threshold_dbm: float) -> pd.DataFrame:
    dataset = run_dir / "ml-ho-dataset.csv"
    if not dataset.exists():
        raise FileNotFoundError(dataset)
    df = pd.read_csv(dataset)
    if df.empty:
        return pd.DataFrame(columns=["time", "coverage_pct", "label"])

    # The candidate dataset repeats each UE over multiple candidate cells.
    # Keep one serving-RSRP sample per UE and RIC decision time.
    serving = (
        df[["time", "ueId", "servingRsrp"]]
        .drop_duplicates(["time", "ueId"])
        .copy()
    )
    coverage = (
        serving.assign(covered=serving["servingRsrp"].ge(threshold_dbm))
        .groupby("time", as_index=False)
        .agg(coverage_pct=("covered", lambda x: 100.0 * float(x.mean())), observed_ues=("ueId", "nunique"))
    )
    coverage["label"] = label
    coverage["run"] = run_dir.name
    return coverage


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--ai-run", type=Path, default=DEFAULT_AI_RUN)
    parser.add_argument("--coverage-rsrp-threshold-dbm", type=float, default=-110.0)
    parser.add_argument("--output", type=Path, default=Path("docs/figures/uav_coverage_rule_vs_ai.pdf"))
    parser.add_argument("--png", type=Path, default=Path("docs/figures/uav_coverage_rule_vs_ai.png"))
    parser.add_argument("--csv", type=Path, default=Path("docs/figures/uav_coverage_rule_vs_ai.csv"))
    args = parser.parse_args()

    style_path = Path("./latex_style.mplstyle")
    if not style_path.exists():
        style_path = Path(__file__).resolve().parents[3] / "latex_style.mplstyle"
    if style_path.exists():
        plt.style.use(style_path)

    frames = []
    for run_dir in sorted(args.dataset_root.glob("*-sat")):
        if run_dir.name.endswith("-no-sat"):
            continue
        frames.append(load_coverage(run_dir, "Rule-based", args.coverage_rsrp_threshold_dbm))
    if args.ai_run.exists():
        frames.append(load_coverage(args.ai_run, "RF-AI", args.coverage_rsrp_threshold_dbm))
    if not frames:
        raise FileNotFoundError("No coverage inputs found")

    data = pd.concat(frames, ignore_index=True)
    summary = (
        data.groupby(["label", "time"], as_index=False)
        .agg(mean=("coverage_pct", "mean"), std=("coverage_pct", "std"), runs=("run", "nunique"))
        .sort_values(["label", "time"])
    )
    summary["std"] = summary["std"].fillna(0.0)
    summary["sem"] = summary["std"] / np.sqrt(summary["runs"].clip(lower=1))

    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "font.size": 14,
            "axes.labelsize": 15,
            "xtick.labelsize": 12,
            "ytick.labelsize": 12,
            "legend.fontsize": 11,
            "lines.markersize": 5,
        }
    )

    fig, ax = plt.subplots(figsize=(8.2, 4.6), constrained_layout=True)
    styles = {
        "Rule-based": ("#F28E2B", "o", "-."),
        "RF-AI": ("#4C78A8", "D", "-"),
    }
    for label, (color, marker, linestyle) in styles.items():
        part = summary[summary["label"].eq(label)]
        if part.empty:
            continue
        ax.plot(
            part["time"],
            part["mean"],
            label=f"{label}",
            color=color,
            marker=marker,
            markevery=max(1, len(part) // 8),
            linestyle=linestyle,
            linewidth=2.2,
        )
        if part["runs"].max() > 1:
            ax.fill_between(
                part["time"],
                (part["mean"] - part["sem"]).clip(lower=0.0),
                (part["mean"] + part["sem"]).clip(upper=100.0),
                color=color,
                alpha=0.14,
                linewidth=0,
            )

    ax.set_xlabel("Simulation time (s)")
    ax.set_ylabel(r"Serving-link coverage (\%)")
    ax.set_xlim(0, 120)
    ax.set_ylim(0, 100)
    ax.grid(True, linestyle="--", alpha=0.42)
    ax.legend(loc="lower right", frameon=True)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output)
    fig.savefig(args.png, dpi=300)
    summary.to_csv(args.csv, index=False)

    print(f"[saved] {args.output}")
    print(f"[saved] {args.png}")
    print(f"[saved] {args.csv}")
    print(summary.groupby("label")["mean"].agg(["mean", "min", "max"]).to_string())


if __name__ == "__main__":
    main()
