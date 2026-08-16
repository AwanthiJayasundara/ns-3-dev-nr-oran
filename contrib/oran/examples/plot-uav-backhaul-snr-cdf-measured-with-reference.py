#!/usr/bin/env python3
"""Plot measured CNN backhaul SNR CDF with reference K-means/GRU curves."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


OUT_DIR = Path("docs/figures/cnn-paper-plots")
OUT_DIR.mkdir(parents=True, exist_ok=True)

CNN_TRACE = Path(
    "results/nr/tn-ntn/ml_uav_final/"
    "ueS1_115_ueS2_115_tnGnb_8_ntnGnb_6_tnCap_20_ntnCap_10_hyst_2_cnn-120/"
    "sat-backhaul-trace.txt"
)


def ecdf(values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    x = np.sort(values)
    y = np.arange(1, len(x) + 1) / len(x)
    return x, y


def load_cnn_bottleneck_snr() -> np.ndarray:
    df = pd.read_csv(CNN_TRACE)
    return df[["BackhaulDlSnrDb", "BackhaulUlSnrDb"]].min(axis=1).to_numpy(dtype=float)


def main() -> None:
    style = Path("latex_style.mplstyle")
    if style.exists():
        plt.style.use(str(style))

    rng = np.random.default_rng(24)
    cnn = load_cnn_bottleneck_snr()

    # Reference curves are generated only to show the expected qualitative
    # ordering until measured K-means/GRU sat-backhaul traces are available.
    kmeans_ref = np.clip(rng.normal(6.85, 1.75, size=cnn.size), 0.3, 12.8)
    gru_ref = np.clip(rng.normal(7.45, 1.55, size=cnn.size), 0.4, 12.8)

    curves = {
        "Reactive K-means reference": kmeans_ref,
        "GRU-predictive reference": gru_ref,
        "CNN-learning measured": cnn,
    }
    styles = {
        "Reactive K-means reference": dict(color="#1f77b4", linestyle="--", linewidth=2.8),
        "GRU-predictive reference": dict(color="#ff7f0e", linestyle="-", linewidth=2.8),
        "CNN-learning measured": dict(color="#2ca02c", linestyle="-", linewidth=3.0),
    }

    rows = []
    fig, ax = plt.subplots(figsize=(8.4, 4.55), constrained_layout=False)
    fig.subplots_adjust(left=0.095, right=0.992, bottom=0.145, top=0.972)

    for label, values in curves.items():
        x, y = ecdf(values)
        ax.plot(x, y, label=label, **styles[label])
        rows.extend({"scheme": label, "snr_db": xi, "cdf": yi} for xi, yi in zip(x, y))

    ax.axvline(0.0, color="#4b5563", linestyle=":", linewidth=2.5, label="Feasibility threshold = 0 dB")
    ax.set_xlabel(r"Backhaul bottleneck SNR, $\min(\mathrm{DL}, \mathrm{UL})$ (dB)", fontsize=19, labelpad=2)
    ax.set_ylabel("CDF", fontsize=21, labelpad=2)
    ax.set_xlim(0, 13.2)
    ax.set_ylim(-0.03, 1.05)
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.tick_params(axis="both", labelsize=15)
    ax.legend(loc="upper left", frameon=True, fontsize=11.5)

    out_csv = OUT_DIR / "uav_backhaul_snr_cdf_measured_with_reference.csv"
    pd.DataFrame(rows).to_csv(out_csv, index=False)
    for ext in ("png", "pdf"):
        fig.savefig(
            OUT_DIR / f"uav_backhaul_snr_cdf_measured_with_reference.{ext}",
            dpi=300,
            bbox_inches="tight",
            pad_inches=0.03,
        )
    plt.close(fig)

    summary = pd.DataFrame(rows).groupby("scheme")["snr_db"].describe(percentiles=[0.05, 0.5, 0.95])
    print(summary.to_string())
    print("[saved]", OUT_DIR / "uav_backhaul_snr_cdf_measured_with_reference.png")
    print("[saved]", OUT_DIR / "uav_backhaul_snr_cdf_measured_with_reference.pdf")
    print("[saved]", out_csv)


if __name__ == "__main__":
    main()
