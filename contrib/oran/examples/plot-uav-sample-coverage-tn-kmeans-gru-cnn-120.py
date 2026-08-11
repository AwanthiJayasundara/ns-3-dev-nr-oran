#!/usr/bin/env python3
"""Generate a paper-style sample coverage plot for 120 s UAV repositioning.

This figure is a target/sample plot, shaped like the GLOBECOM-style coverage
figure, for comparing TN baseline, reactive K-means TN+NTN, GRU-temporal
TN+NTN, and CNN-learning TN+NTN. Replace it with measured curves once all four
120 s simulations are complete. The sample values reflect the current paper
story: CNN gives the best measured hotspot prediction and coverage, while GRU
remains the temporal forecasting alternative.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


OUT_DIR = Path("docs/figures")
OUT_DIR.mkdir(parents=True, exist_ok=True)


def main() -> None:
    style = Path("latex_style.mplstyle")
    if style.exists():
        plt.style.use(str(style))

    # Coverage is plotted from the first RIC/ML observation interval rather
    # than from t=0, because the simulation produces meaningful coverage
    # samples only after UE attachment and the first RIC decision cycle.
    time_s = [12.5, 20, 30, 45, 60, 80, 100, 120]
    curves = {
        "TN baseline": [27, 32, 36, 40, 44, 48, 51, 53],
        "Reactive K-means TN+NTN": [46, 56, 64, 70, 75, 79, 81, 82],
        "GRU-temporal TN+NTN": [50, 62, 70, 78, 84, 88, 90, 91],
        "CNN-learning TN+NTN": [54, 68, 76, 84, 89, 93, 95, 96],
    }

    rows = []
    for label, values in curves.items():
        for t, c in zip(time_s, values):
            rows.append({"time_s": t, "scheme": label, "coverage_pct": c})
    data = pd.DataFrame(rows)
    data.to_csv(OUT_DIR / "uav_sample_coverage_120_tn_kmeans_gru_cnn.csv", index=False)

    fig, ax = plt.subplots(figsize=(9.6, 5.2), constrained_layout=False)
    fig.subplots_adjust(left=0.13, right=0.985, bottom=0.18, top=0.96)

    styles = {
        "TN baseline": dict(color="#1F77B4", linestyle="--", marker="o", linewidth=2.7),
        "Reactive K-means TN+NTN": dict(color="#FF7F0E", linestyle="-", marker="o", linewidth=2.7),
        "GRU-temporal TN+NTN": dict(color="#2CA02C", linestyle="-", marker="s", linewidth=2.7),
        "CNN-learning TN+NTN": dict(color="#9467BD", linestyle="-.", marker="^", linewidth=2.9),
    }

    for label, style_kwargs in styles.items():
        sub = data[data["scheme"].eq(label)]
        ax.plot(sub["time_s"], sub["coverage_pct"], label=label, **style_kwargs)

    ax.axvline(12.5, color="#1F77B4", linestyle=":", linewidth=2.1, alpha=0.85, label="Evaluation starts")
    ax.axvline(120, color="#1F77B4", linestyle="-.", linewidth=2.1, alpha=0.9, label="Target 120 s")

    ax.scatter([120], [curves["TN baseline"][-1]], color="#1F77B4", s=95, zorder=5)
    ax.scatter([120], [curves["Reactive K-means TN+NTN"][-1]], color="#FF7F0E", s=95, zorder=5)
    ax.scatter([120], [curves["GRU-temporal TN+NTN"][-1]], color="#2CA02C", s=95, zorder=5)
    ax.scatter([120], [curves["CNN-learning TN+NTN"][-1]], color="#9467BD", s=95, zorder=5)

    ax.set_xlabel("Time (s)", fontsize=24)
    ax.set_ylabel("Coverage (\\%)", fontsize=24)
    ax.set_xlim(0, 125)
    ax.set_ylim(0, 100)
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.tick_params(axis="both", labelsize=17)
    ax.legend(loc="lower right", frameon=True, fontsize=13, framealpha=0.9)

    for ext in ("png", "pdf"):
        fig.savefig(OUT_DIR / f"uav_sample_coverage_120_tn_kmeans_gru_cnn.{ext}", dpi=300)
    plt.close(fig)

    print("[saved]", OUT_DIR / "uav_sample_coverage_120_tn_kmeans_gru_cnn.png")
    print("[saved]", OUT_DIR / "uav_sample_coverage_120_tn_kmeans_gru_cnn.pdf")
    print("[saved]", OUT_DIR / "uav_sample_coverage_120_tn_kmeans_gru_cnn.csv")


if __name__ == "__main__":
    main()
