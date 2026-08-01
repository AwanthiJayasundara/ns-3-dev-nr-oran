#!/usr/bin/env python3
"""Plot measured delivered-packet DL delay for current UAV TN/NTN runs."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


DEFAULT_DATASET_ROOT = Path("results/nr/tn-ntn/ai-dataset-v2")
DEFAULT_AI_RUN = Path(
    "results/nr/tn-ntn/"
    "tn-uav-satellite_ueS1_50_ueS2_50_tnGnb_4_ntnGnb_3_tnCap_20_ntnCap_10_hyst_2_ai-switching-sat"
)


def load_style() -> None:
    style_path = Path("./latex_style.mplstyle")
    if not style_path.exists():
        style_path = Path(__file__).resolve().parents[3] / "latex_style.mplstyle"
    if style_path.exists():
        plt.style.use(style_path)


def seed_from_name(path: Path) -> int:
    match = re.search(r"seed(\d+)", path.name)
    return int(match.group(1)) if match else -1


def load_delay(result_dir: Path, label: str) -> pd.DataFrame:
    qos_file = result_dir / "qos-vs-time.txt"
    if not qos_file.exists():
        raise FileNotFoundError(qos_file)

    df = pd.read_csv(qos_file)
    df = df[df["Dir"].eq("DL")].copy()
    df = df[df["Delay"].gt(0.0)].copy()
    if df.empty:
        return pd.DataFrame(columns=["Time", "delay_ms", "label", "seed"])

    # Delay in qos-vs-time.txt is stored in seconds.
    series = (
        df.assign(delay_ms=df["Delay"] * 1000.0)
        .groupby("Time", as_index=False)["delay_ms"]
        .mean()
    )
    series["label"] = label
    series["seed"] = seed_from_name(result_dir)
    return series


def load_group(dataset_root: Path, suffix: str, label: str) -> pd.DataFrame:
    dirs = sorted(dataset_root.glob(f"*seed*{suffix}"))
    if suffix == "-sat":
        dirs = [d for d in dirs if d.name.endswith("-sat") and not d.name.endswith("-no-sat")]
    else:
        dirs = [d for d in dirs if d.name.endswith(suffix)]
    if not dirs:
        raise FileNotFoundError(f"No result folders found for suffix {suffix} in {dataset_root}")
    return pd.concat([load_delay(d, label) for d in dirs], ignore_index=True)


def summarize(data: pd.DataFrame) -> pd.DataFrame:
    if data.empty:
        return pd.DataFrame(columns=["label", "Time", "mean", "std", "count", "sem"])
    grouped = data.groupby(["label", "Time"])["delay_ms"]
    out = grouped.agg(mean="mean", std="std", count="count").reset_index()
    out["std"] = out["std"].fillna(0.0)
    out["sem"] = out["std"] / np.sqrt(out["count"].clip(lower=1))
    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--ai-run", type=Path, default=DEFAULT_AI_RUN)
    parser.add_argument("--output", type=Path, default=Path("docs/figures/uav_xhaul_delay_over_time.pdf"))
    parser.add_argument("--png", type=Path, default=Path("docs/figures/uav_xhaul_delay_over_time.png"))
    parser.add_argument("--csv", type=Path, default=Path("docs/figures/uav_xhaul_delay_over_time.csv"))
    parser.add_argument(
        "--delay-cap-ms",
        type=float,
        default=750.0,
        help="Cap plotted mean delay values to keep isolated spikes readable. Use <=0 to disable.",
    )
    args = parser.parse_args()

    load_style()
    frames = [
        load_group(args.dataset_root, "-no-sat", "TN + UAV, no satellite"),
        load_group(args.dataset_root, "-sat", "TN + UAV + satellite"),
    ]
    if args.ai_run.exists():
        frames.append(load_delay(args.ai_run, "TN + UAV + satellite, AI"))

    data = pd.concat(frames, ignore_index=True)
    if args.delay_cap_ms > 0.0 and not data.empty:
        data["delay_ms"] = data["delay_ms"].clip(upper=args.delay_cap_ms)
    summary = summarize(data)

    styles = {
        "TN + UAV, no satellite": ("#E15759", "s", "--"),
        "TN + UAV + satellite": ("#F28E2B", "o", "-."),
        "TN + UAV + satellite, AI": ("#4C78A8", "D", "-"),
    }

    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "font.size": 16,
            "axes.labelsize": 18,
            "xtick.labelsize": 15,
            "ytick.labelsize": 15,
            "legend.fontsize": 13,
            "lines.markersize": 6,
        }
    )
    fig, ax = plt.subplots(figsize=(7.8, 4.8), constrained_layout=True)

    for label, (color, marker, linestyle) in styles.items():
        part = summary[summary["label"].eq(label)].sort_values("Time")
        if part.empty:
            continue
        ax.plot(
            part["Time"],
            part["mean"],
            label=label,
            color=color,
            marker=marker,
            linestyle=linestyle,
            linewidth=2.4,
            markevery=max(1, len(part) // 10),
        )
        if part["count"].max() > 1:
            ax.fill_between(
                part["Time"],
                (part["mean"] - part["sem"]).clip(lower=0.0),
                part["mean"] + part["sem"],
                color=color,
                alpha=0.12,
                linewidth=0,
            )

    ax.axvspan(30, 75, color="#C44E52", alpha=0.10)
    ax.axvline(30, color="#C44E52", linestyle=":", linewidth=1.4)
    ax.axvline(75, color="#C44E52", linestyle=":", linewidth=1.4)
    ax.set_xlabel("Simulation time (s)")
    ax.set_ylabel("DL delay (ms)")
    ax.set_xlim(0, 120)
    if args.delay_cap_ms > 0.0:
        ax.set_ylim(0, args.delay_cap_ms * 1.08)
    ax.grid(True, linestyle="--", alpha=0.45)
    ax.legend(loc="upper right", frameon=True)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output)
    fig.savefig(args.png, dpi=300)
    summary.to_csv(args.csv, index=False)

    print(f"[saved] {args.output}")
    print(f"[saved] {args.png}")
    print(f"[saved] {args.csv}")
    for label, part in summary.groupby("label"):
        print(
            f"[summary] {label}: mean={part['mean'].mean():.3f} ms, "
            f"max={part['mean'].max():.3f} ms, time-points={len(part)}"
        )


if __name__ == "__main__":
    main()
