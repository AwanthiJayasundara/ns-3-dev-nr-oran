#!/usr/bin/env python3
"""Compare UAV donor path-loss settings using ai-dataset-v1 traces."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


DEFAULT_DATASET_ROOT = Path("results/nr/tn-ntn/ai-dataset-v1")

MODEL_STYLE = {
    "fspl": ("FSPL trace", "#4C78A8", "o"),
    "logdist": ("Log-distance trace", "#F28E2B", "s"),
    "logdist-fading": ("Log-distance + fading trace", "#59A14F", "^"),
}


def fspl_db(frequency_hz: float, distance_m: np.ndarray) -> np.ndarray:
    f_ghz = frequency_hz / 1e9
    d = np.maximum(distance_m, 1.0)
    return 32.4 + 20.0 * np.log10(f_ghz) + 20.0 * np.log10(d)


def log_distance_pathloss_db(
    frequency_hz: float,
    distance_m: np.ndarray,
    exponent: float,
    reference_distance_m: float,
) -> np.ndarray:
    d0 = max(reference_distance_m, 1.0)
    d = np.maximum(distance_m, 1.0)
    l0 = fspl_db(frequency_hz, np.array([d0]))[0]
    return np.where(d <= d0, l0, l0 + 10.0 * exponent * np.log10(d / d0))


def infer_model_name(run_name: str) -> str:
    if "logdist-fading" in run_name:
        return "logdist-fading"
    if "logdist" in run_name:
        return "logdist"
    if "fspl" in run_name:
        return "fspl"
    return "unknown"


def load_dataset(dataset_root: Path, tx_power_dbm: float) -> pd.DataFrame:
    frames = []
    for trace in sorted(dataset_root.glob("*/xhaul-autonomy-trace.csv")):
        model = infer_model_name(trace.parent.name)
        if model == "unknown":
            continue
        df = pd.read_csv(trace)
        df = df[df["BestDonorDistanceM"].gt(0)].copy()
        if df.empty:
            continue
        df["Run"] = trace.parent.name
        df["LossModel"] = model
        df["EffectivePathLossDb"] = tx_power_dbm - df["XhaulRsrpDbm"]
        frames.append(df)
    if not frames:
        raise FileNotFoundError(f"No usable xhaul-autonomy-trace.csv files under {dataset_root}")
    return pd.concat(frames, ignore_index=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--xhaul-tx-power-dbm", type=float, default=46.0)
    parser.add_argument("--frequency-hz", type=float, default=4.0e9)
    parser.add_argument("--reference-distance-m", type=float, default=100.0)
    parser.add_argument("--threshold-dbm", type=float, default=-100.0)
    parser.add_argument("--output", type=Path, default=Path("docs/figures/uav_xhaul_pathloss_model_comparison.pdf"))
    parser.add_argument("--png", type=Path, default=Path("docs/figures/uav_xhaul_pathloss_model_comparison.png"))
    parser.add_argument("--csv", type=Path, default=Path("docs/figures/uav_xhaul_pathloss_model_comparison.csv"))
    args = parser.parse_args()

    style_path = Path("./latex_style.mplstyle")
    if not style_path.exists():
        style_path = Path(__file__).resolve().parents[3] / "latex_style.mplstyle"
    if style_path.exists():
        plt.style.use(style_path)
    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "font.size": 14,
            "axes.labelsize": 15,
            "xtick.labelsize": 12,
            "ytick.labelsize": 12,
            "legend.fontsize": 10,
        }
    )

    data = load_dataset(args.dataset_root, args.xhaul_tx_power_dbm)
    d_min = max(args.reference_distance_m, data["BestDonorDistanceM"].min() * 0.9)
    d_max = data["BestDonorDistanceM"].max() * 1.06
    distance_grid = np.linspace(d_min, d_max, 300)

    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(8.0, 6.4), sharex=True, constrained_layout=True)

    for model, (label, color, marker) in MODEL_STYLE.items():
        part = data[data["LossModel"].eq(model)]
        if part.empty:
            continue
        ax0.scatter(
            part["BestDonorDistanceM"] / 1000.0,
            part["EffectivePathLossDb"],
            s=24,
            color=color,
            alpha=0.60,
            marker=marker,
            edgecolors="none",
            label=label,
        )
        ax1.scatter(
            part["BestDonorDistanceM"] / 1000.0,
            part["XhaulRsrpDbm"],
            s=24,
            color=color,
            alpha=0.60,
            marker=marker,
            edgecolors="none",
            label=label,
        )

    model_curves = [
        (r"FSPL model ($\gamma=2.0$)", fspl_db(args.frequency_hz, distance_grid), "#4C78A8", "--"),
        (
            r"Log-distance model ($\gamma=4.8$)",
            log_distance_pathloss_db(args.frequency_hz, distance_grid, 4.8, args.reference_distance_m),
            "#9467BD",
            "-.",
        ),
        (
            r"Log-distance model ($\gamma=5.6$)",
            log_distance_pathloss_db(args.frequency_hz, distance_grid, 5.6, args.reference_distance_m),
            "black",
            "-",
        ),
    ]
    for label, pathloss, color, linestyle in model_curves:
        ax0.plot(distance_grid / 1000.0, pathloss, color=color, linestyle=linestyle, linewidth=2.0, label=label)
        rsrp = args.xhaul_tx_power_dbm - pathloss
        ax1.plot(distance_grid / 1000.0, rsrp, color=color, linestyle=linestyle, linewidth=2.0, label=label)

    ax0.set_ylabel("Effective path loss (dB)")
    ax0.grid(True, linestyle="--", alpha=0.38)
    handles0, labels0 = ax0.get_legend_handles_labels()
    trace_handles0 = handles0[: len(MODEL_STYLE)]
    trace_labels0 = labels0[: len(MODEL_STYLE)]
    model_handles0 = handles0[len(MODEL_STYLE) :]
    model_labels0 = labels0[len(MODEL_STYLE) :]
    legend_trace0 = ax0.legend(
        trace_handles0,
        trace_labels0,
        loc="upper left",
        frameon=True,
        title="Simulation samples",
        fontsize=9,
        title_fontsize=9,
    )
    ax0.add_artist(legend_trace0)
    ax0.legend(
        model_handles0,
        model_labels0,
        loc="lower right",
        frameon=True,
        title="Theoretical curves",
        fontsize=9,
        title_fontsize=9,
    )

    ax1.axhline(args.threshold_dbm, color="#C44E52", linestyle=":", linewidth=1.8, label=f"{args.threshold_dbm:.0f} dBm threshold")
    ax1.set_xlabel("UAV-to-TN donor distance (km)")
    ax1.set_ylabel("Donor-link RSRP (dBm)")
    ax1.grid(True, linestyle="--", alpha=0.38)
    handles1, labels1 = ax1.get_legend_handles_labels()
    trace_count = len([m for m in MODEL_STYLE if not data[data["LossModel"].eq(m)].empty])
    trace_handles1 = handles1[:trace_count]
    trace_labels1 = labels1[:trace_count]
    model_handles1 = handles1[trace_count:]
    model_labels1 = labels1[trace_count:]
    legend_trace1 = ax1.legend(
        trace_handles1,
        trace_labels1,
        loc="lower left",
        frameon=True,
        title="Simulation samples",
        fontsize=9,
        title_fontsize=9,
    )
    ax1.add_artist(legend_trace1)
    ax1.legend(
        model_handles1,
        model_labels1,
        loc="upper right",
        frameon=True,
        title="Theoretical curves",
        fontsize=9,
        title_fontsize=9,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output)
    fig.savefig(args.png, dpi=300)
    data[
        [
            "Run",
            "LossModel",
            "Time",
            "UavIndex",
            "BestDonorDistanceM",
            "EffectivePathLossDb",
            "XhaulRsrpDbm",
            "BackhaulMode",
        ]
    ].to_csv(args.csv, index=False)

    print(f"[saved] {args.output}")
    print(f"[saved] {args.png}")
    print(f"[saved] {args.csv}")
    for model, part in data.groupby("LossModel"):
        print(
            f"[summary] {model}: samples={len(part)}, distance={part['BestDonorDistanceM'].min()/1000:.2f}-"
            f"{part['BestDonorDistanceM'].max()/1000:.2f} km, RSRP mean={part['XhaulRsrpDbm'].mean():.2f} dBm"
        )


if __name__ == "__main__":
    main()
