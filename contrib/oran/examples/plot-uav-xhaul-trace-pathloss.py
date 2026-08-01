#!/usr/bin/env python3
"""Plot trace-based UAV donor path-loss variation from ai-dataset-v2."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


DEFAULT_DATASET_ROOT = Path("results/nr/tn-ntn/ai-dataset-v2")

MODE_COLORS = {
    "TN_DIRECT": "#59A14F",
    "SATELLITE_FALLBACK": "#F28E2B",
    "NO_BACKHAUL_AVAILABLE": "#E15759",
}

MODE_LABELS = {
    "TN_DIRECT": "TN direct",
    "SATELLITE_FALLBACK": "Satellite fallback",
    "NO_BACKHAUL_AVAILABLE": "No backhaul",
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


def load_traces(dataset_root: Path) -> pd.DataFrame:
    frames = []
    for trace in sorted(dataset_root.glob("*-sat/xhaul-autonomy-trace.csv")):
        if trace.parent.name.endswith("-no-sat"):
            continue
        df = pd.read_csv(trace)
        df["Run"] = trace.parent.name
        frames.append(df)
    if not frames:
        raise FileNotFoundError(f"No satellite xhaul-autonomy-trace.csv files found in {dataset_root}")
    return pd.concat(frames, ignore_index=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--xhaul-tx-power-dbm", type=float, default=46.0)
    parser.add_argument("--frequency-hz", type=float, default=4.0e9)
    parser.add_argument("--pathloss-exponent", type=float, default=5.6)
    parser.add_argument("--reference-distance-m", type=float, default=100.0)
    parser.add_argument("--threshold-dbm", type=float, default=-100.0)
    parser.add_argument("--output", type=Path, default=Path("docs/figures/uav_xhaul_trace_pathloss_variation.pdf"))
    parser.add_argument("--png", type=Path, default=Path("docs/figures/uav_xhaul_trace_pathloss_variation.png"))
    parser.add_argument("--csv", type=Path, default=Path("docs/figures/uav_xhaul_trace_pathloss_variation.csv"))
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

    data = load_traces(args.dataset_root)
    data = data[data["BestDonorDistanceM"].gt(0)].copy()
    data["EffectivePathLossDb"] = args.xhaul_tx_power_dbm - data["XhaulRsrpDbm"]
    data["ModelPathLossDb"] = log_distance_pathloss_db(
        args.frequency_hz,
        data["BestDonorDistanceM"].to_numpy(),
        args.pathloss_exponent,
        args.reference_distance_m,
    )
    data["PathLossResidualDb"] = data["EffectivePathLossDb"] - data["ModelPathLossDb"]

    d_min = max(args.reference_distance_m, data["BestDonorDistanceM"].min() * 0.92)
    d_max = data["BestDonorDistanceM"].max() * 1.05
    distance_grid = np.linspace(d_min, d_max, 300)
    model_grid = log_distance_pathloss_db(
        args.frequency_hz,
        distance_grid,
        args.pathloss_exponent,
        args.reference_distance_m,
    )

    fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(7.9, 6.4), sharex=True, constrained_layout=True)

    for mode, color in MODE_COLORS.items():
        part = data[data["BackhaulMode"].eq(mode)]
        if part.empty:
            continue
        ax0.scatter(
            part["BestDonorDistanceM"] / 1000.0,
            part["EffectivePathLossDb"],
            s=24,
            color=color,
            alpha=0.68,
            edgecolors="none",
            label=MODE_LABELS[mode],
        )
        ax1.scatter(
            part["BestDonorDistanceM"] / 1000.0,
            part["XhaulRsrpDbm"],
            s=24,
            color=color,
            alpha=0.68,
            edgecolors="none",
            label=MODE_LABELS[mode],
        )

    ax0.plot(
        distance_grid / 1000.0,
        model_grid,
        color="black",
        linewidth=2.0,
        label=fr"Log-distance model, $\gamma={args.pathloss_exponent}$",
    )
    ax0.set_ylabel("Effective path loss (dB)")
    ax0.grid(True, linestyle="--", alpha=0.38)
    ax0.legend(loc="upper left", frameon=True, ncol=2)

    ax1.axhline(
        args.threshold_dbm,
        color="#C44E52",
        linestyle=":",
        linewidth=1.8,
        label=f"{args.threshold_dbm:.0f} dBm threshold",
    )
    ax1.set_xlabel("UAV-to-TN donor distance (km)")
    ax1.set_ylabel("Donor-link RSRP (dBm)")
    ax1.grid(True, linestyle="--", alpha=0.38)
    ax1.legend(loc="upper right", frameon=True, ncol=2)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output)
    fig.savefig(args.png, dpi=300)
    data[
        [
            "Run",
            "Time",
            "UavIndex",
            "BestDonorDistanceM",
            "EffectivePathLossDb",
            "ModelPathLossDb",
            "PathLossResidualDb",
            "XhaulRsrpDbm",
            "BackhaulMode",
        ]
    ].to_csv(args.csv, index=False)

    print(f"[saved] {args.output}")
    print(f"[saved] {args.png}")
    print(f"[saved] {args.csv}")
    print(
        "[summary] samples={}, distance={:.2f}-{:.2f} km, pathloss residual mean={:.2f} dB std={:.2f} dB".format(
            len(data),
            data["BestDonorDistanceM"].min() / 1000.0,
            data["BestDonorDistanceM"].max() / 1000.0,
            data["PathLossResidualDb"].mean(),
            data["PathLossResidualDb"].std(),
        )
    )


if __name__ == "__main__":
    main()
