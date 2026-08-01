#!/usr/bin/env python3
"""Plot measured DL PDR over time for UAV TN/NTN switching runs."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path("results/nr/tn-ntn")
DATASET_ROOT = ROOT / "ai-dataset-v2"
AI_RUN = ROOT / "tn-uav-satellite_ueS1_50_ueS2_50_tnGnb_4_ntnGnb_3_tnCap_20_ntnCap_10_hyst_2_ai-switching-sat"


def load_style() -> None:
    style_path = Path("./latex_style.mplstyle")
    if not style_path.exists():
        style_path = Path(__file__).resolve().parents[3] / "latex_style.mplstyle"
    if style_path.exists():
        plt.style.use(style_path)


def seed_from_name(path: Path) -> int:
    match = re.search(r"seed(\d+)", path.name)
    return int(match.group(1)) if match else -1


def load_qos_pdr(result_dir: Path, label: str) -> pd.DataFrame:
    qos_file = result_dir / "qos-vs-time.txt"
    if not qos_file.exists():
        raise FileNotFoundError(qos_file)

    df = pd.read_csv(qos_file)
    df = df[df["Dir"].eq("DL")].copy()
    if df.empty:
        raise ValueError(f"No DL rows in {qos_file}")

    # PDR is already stored as a percentage in qos-vs-time.txt.
    series = (
        df.groupby("Time", as_index=False)["PDR"]
        .mean()
        .rename(columns={"PDR": "pdr"})
    )
    series["label"] = label
    series["seed"] = seed_from_name(result_dir)
    return series


def load_seed_group(
    dataset_root: Path,
    pattern: str,
    label: str,
    suffix: str | None = None,
) -> pd.DataFrame:
    dirs = sorted(p for p in dataset_root.glob(pattern) if p.is_dir())
    if suffix == "-sat":
        dirs = [p for p in dirs if p.name.endswith("-sat") and not p.name.endswith("-no-sat")]
    elif suffix is not None:
        dirs = [p for p in dirs if p.name.endswith(suffix)]
    if not dirs:
        raise FileNotFoundError(f"No result directories match {dataset_root / pattern}")
    return pd.concat([load_qos_pdr(d, label) for d in dirs], ignore_index=True)


def summarize_group(df: pd.DataFrame) -> pd.DataFrame:
    grouped = df.groupby(["label", "Time"])["pdr"]
    out = grouped.agg(mean="mean", std="std", count="count").reset_index()
    out["std"] = out["std"].fillna(0.0)
    out["sem"] = out["std"] / np.sqrt(out["count"].clip(lower=1))
    return out


def smooth(values: pd.Series, window: int) -> pd.Series:
    if window <= 1:
        return values
    return values.rolling(window=window, min_periods=1, center=True).mean()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, default=DATASET_ROOT)
    parser.add_argument("--ai-run", type=Path, default=AI_RUN)
    parser.add_argument("--output", type=Path, default=Path("docs/figures/uav_xhaul_pdr_over_time.pdf"))
    parser.add_argument("--png", type=Path, default=Path("docs/figures/uav_xhaul_pdr_over_time.png"))
    parser.add_argument("--csv", type=Path, default=Path("docs/figures/uav_xhaul_pdr_over_time.csv"))
    parser.add_argument("--smooth-window", type=int, default=3)
    parser.add_argument(
        "--pdr-offset-percent",
        type=float,
        default=0.0,
        help="Optional display offset added to each measured PDR value, in percentage points.",
    )
    args = parser.parse_args()

    load_style()
    no_sat = load_seed_group(
        args.dataset_root,
        "*seed*-no-sat",
        "TN + UAV, no satellite",
        suffix="-no-sat",
    )
    sat_rule = load_seed_group(
        args.dataset_root,
        "*seed*-sat",
        "TN + UAV + satellite",
        suffix="-sat",
    )
    measured = [no_sat, sat_rule]

    if args.ai_run.exists():
        measured.append(load_qos_pdr(args.ai_run, "TN + UAV + satellite, AI"))

    data = pd.concat(measured, ignore_index=True)
    if args.pdr_offset_percent:
        data["pdr"] = (data["pdr"] + args.pdr_offset_percent).clip(upper=100.0)
    summary = summarize_group(data)

    styles = {
        "TN + UAV, no satellite": ("#E15759", "s", "--"),
        "TN + UAV + satellite": ("#F28E2B", "o", "-."),
        "TN + UAV + satellite, AI": ("#4C78A8", "D", "-"),
    }

    fig, ax = plt.subplots(figsize=(8.0, 4.7))
    for label, (color, marker, linestyle) in styles.items():
        part = summary[summary["label"].eq(label)].sort_values("Time")
        if part.empty:
            continue
        y = smooth(part["mean"], args.smooth_window)
        ax.plot(
            part["Time"],
            y,
            label=label,
            color=color,
            marker=marker,
            markevery=max(1, len(part) // 10),
            markersize=4.5,
            linewidth=1.9,
            linestyle=linestyle,
        )
        if part["count"].max() > 1:
            lower = smooth((part["mean"] - part["sem"]).clip(lower=0.0), args.smooth_window)
            upper = smooth(part["mean"] + part["sem"], args.smooth_window)
            ax.fill_between(part["Time"], lower, upper, color=color, alpha=0.12, linewidth=0)

    ax.set_xlabel("Simulation time (s)")
    ylabel = r"DL PDR (\%)"
    if args.pdr_offset_percent:
        ylabel = r"Adjusted DL PDR (\%)"
    ax.set_ylabel(ylabel)
    ax.set_xlim(0, 120)
    ymax = max(1.0, float(summary["mean"].max()) * 1.25)
    ax.set_ylim(0, ymax)
    ax.grid(True, linestyle="--", alpha=0.45)
    ax.legend(loc="upper right", frameon=True)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(args.output)
    fig.savefig(args.png, dpi=300)
    summary.to_csv(args.csv, index=False)
    print(f"[saved] {args.output}")
    print(f"[saved] {args.png}")
    print(f"[saved] {args.csv}")


if __name__ == "__main__":
    main()
