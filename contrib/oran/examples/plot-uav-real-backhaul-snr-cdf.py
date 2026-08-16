#!/usr/bin/env python3
"""Plot measured satellite-backhaul bottleneck SNR CDF from ns-3 traces."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


OUT_DIR = Path("docs/figures/cnn-paper-plots")
OUT_DIR.mkdir(parents=True, exist_ok=True)

RUNS = {
    "Reactive K-means": Path(
        "results/nr/tn-ntn/ml_uav_final/"
        "ueS1_115_ueS2_115_tnGnb_8_ntnGnb_6_tnCap_20_ntnCap_10_hyst_2_kmeans-120/"
        "sat-backhaul-trace.txt"
    ),
    "GRU-predictive": Path(
        "results/nr/tn-ntn/ml_uav_final/"
        "ueS1_115_ueS2_115_tnGnb_8_ntnGnb_6_tnCap_20_ntnCap_10_hyst_2_gru/"
        "sat-backhaul-trace.txt"
    ),
    "CNN-learning": Path(
        "results/nr/tn-ntn/ml_uav_final/"
        "ueS1_115_ueS2_115_tnGnb_8_ntnGnb_6_tnCap_20_ntnCap_10_hyst_2_cnn-120/"
        "sat-backhaul-trace.txt"
    ),
}


def load_bottleneck_snr(path: Path) -> np.ndarray:
    if not path.exists() or path.stat().st_size == 0:
        return np.array([], dtype=float)
    df = pd.read_csv(path)
    if df.empty:
        return np.array([], dtype=float)
    return df[["BackhaulDlSnrDb", "BackhaulUlSnrDb"]].min(axis=1).to_numpy(dtype=float)


def main() -> None:
    style = Path("latex_style.mplstyle")
    if style.exists():
        plt.style.use(str(style))

    rows = []
    fig, ax = plt.subplots(figsize=(8.6, 4.8), constrained_layout=False)
    fig.subplots_adjust(left=0.12, right=0.985, bottom=0.18, top=0.965)

    styles = {
        "Reactive K-means": dict(color="#1f77b4", linestyle="--", linewidth=2.8),
        "GRU-predictive": dict(color="#ff7f0e", linestyle="-", linewidth=2.8),
        "CNN-learning": dict(color="#2ca02c", linestyle="-", linewidth=2.8),
    }

    plotted = 0
    for label, path in RUNS.items():
        values = load_bottleneck_snr(path)
        if values.size == 0:
            print(f"[skip] {label}: no measured sat-backhaul trace at {path}")
            continue
        x = np.sort(values)
        y = np.arange(1, len(x) + 1) / len(x)
        ax.plot(x, y, label=f"{label} (measured)", **styles[label])
        rows.extend({"scheme": label, "snr_db": xi, "cdf": yi} for xi, yi in zip(x, y))
        plotted += 1

    if plotted == 0:
        raise SystemExit("No non-empty sat-backhaul traces found.")

    ax.axvline(0.0, color="#4b5563", linestyle=":", linewidth=2.5, label="Feasibility threshold = 0 dB")
    ax.set_xlabel(r"Backhaul bottleneck SNR, $\min(\mathrm{DL}, \mathrm{UL})$ (dB)", fontsize=20)
    ax.set_ylabel("CDF", fontsize=22)
    ax.set_ylim(-0.03, 1.05)
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.tick_params(axis="both", labelsize=15)
    ax.legend(loc="upper left", frameon=True, fontsize=12)

    out_csv = OUT_DIR / "uav_real_backhaul_snr_cdf.csv"
    pd.DataFrame(rows).to_csv(out_csv, index=False)
    for ext in ("png", "pdf"):
        fig.savefig(OUT_DIR / f"uav_real_backhaul_snr_cdf.{ext}", dpi=300)
    plt.close(fig)

    summary = pd.DataFrame(rows).groupby("scheme")["snr_db"].describe(percentiles=[0.05, 0.5, 0.95])
    print(summary.to_string())
    print("[saved]", OUT_DIR / "uav_real_backhaul_snr_cdf.png")
    print("[saved]", OUT_DIR / "uav_real_backhaul_snr_cdf.pdf")
    print("[saved]", out_csv)


if __name__ == "__main__":
    main()
