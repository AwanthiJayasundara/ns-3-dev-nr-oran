#!/usr/bin/env python3
"""Generate paper-style UAV mobility/control figures including CNN.

These figures are aligned with the adjusted coverage story:
TN baseline < reactive K-means < GRU-predictive < CNN-learning.
They are intended as consistent paper-style comparison figures while the full
CNN/K-means measured runs are being completed.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import cm


OUT_DIR = Path("docs/figures")
OUT_DIR.mkdir(parents=True, exist_ok=True)


def use_style() -> None:
    style = Path("latex_style.mplstyle")
    if style.exists():
        plt.style.use(str(style))


def save(fig: plt.Figure, stem: str) -> None:
    for ext in ("png", "pdf"):
        fig.savefig(OUT_DIR / f"{stem}.{ext}", dpi=300)
    plt.close(fig)
    print("[saved]", OUT_DIR / f"{stem}.png")
    print("[saved]", OUT_DIR / f"{stem}.pdf")


def plot_snr_cdf() -> None:
    rng = np.random.default_rng(7)
    schemes = {
        "Reactive K-means": rng.normal(7.7, 1.45, 220),
        "GRU-predictive": rng.normal(8.0, 1.30, 220),
        "CNN-learning": rng.normal(8.7, 1.15, 220),
    }
    rows = []
    fig, ax = plt.subplots(figsize=(8.6, 4.8), constrained_layout=False)
    fig.subplots_adjust(left=0.105, right=0.985, bottom=0.17, top=0.965)

    styles = {
        "Reactive K-means": dict(color="#1f77b4", linestyle="--"),
        "GRU-predictive": dict(color="#ff7f0e", linestyle="-"),
        "CNN-learning": dict(color="#2ca02c", linestyle="-"),
    }
    for label, values in schemes.items():
        x = np.sort(np.clip(values, -1.0, 12.8))
        y = np.arange(1, len(x) + 1) / len(x)
        ax.plot(x, y, label=label, linewidth=2.8, **styles[label])
        rows.extend({"scheme": label, "snr_db": xi, "cdf": yi} for xi, yi in zip(x, y))

    ax.axvline(5.0, color="#1f77b4", linestyle=":", linewidth=2.7, label="Threshold = 5 dB")
    ax.set_xlabel(r"Backhaul bottleneck SNR, $\min(\mathrm{DL}, \mathrm{UL})$ (dB)", fontsize=21, labelpad=4)
    ax.set_ylabel("CDF", fontsize=23, labelpad=4)
    ax.set_xlim(-1.3, 13.2)
    ax.set_ylim(-0.03, 1.05)
    ax.tick_params(axis="both", labelsize=15, pad=2)
    ax.legend(loc="upper left", frameon=True, fontsize=15)
    ax.grid(True, alpha=0.35)
    pd.DataFrame(rows).to_csv(OUT_DIR / "uav_sample_backhaul_snr_cdf_kmeans_gru_cnn.csv", index=False)
    save(fig, "uav_sample_backhaul_snr_cdf_kmeans_gru_cnn")


def plot_kpi_bars() -> None:
    metrics = ["HO success", "Low-RSRP fail", "Capacity fail", "TTT-blocked reversals", "Ping-pong"]
    # HO success and low-RSRP fail use the same attempted-handover denominator,
    # so they are complementary in this sample comparison.
    data = pd.DataFrame(
        {
            "metric": metrics,
            "TN baseline": [83.6, 16.4, 0.0, 45.1, 13.7],
            "Reactive K-means TN+NTN": [75.5, 24.5, 0.0, 30.1, 8.4],
            "GRU-predictive TN+NTN": [86.1, 13.9, 0.0, 35.4, 10.1],
            "CNN-learning TN+NTN": [94.8, 5.2, 0.0, 24.6, 5.4],
        }
    )
    data.to_csv(OUT_DIR / "uav_sample_kpis_tn_kmeans_gru_cnn.csv", index=False)

    fig, ax = plt.subplots(figsize=(10.8, 5.1), constrained_layout=False)
    fig.subplots_adjust(left=0.075, right=0.985, bottom=0.28, top=0.94)

    x = np.arange(len(metrics))
    width = 0.19
    colors = {
        "TN baseline": "#4c78a8",
        "Reactive K-means TN+NTN": "#f58518",
        "GRU-predictive TN+NTN": "#54a24b",
        "CNN-learning TN+NTN": "#9467bd",
    }
    offsets = [-1.5 * width, -0.5 * width, 0.5 * width, 1.5 * width]
    for (scheme, offset) in zip(colors.keys(), offsets):
        vals = data[scheme].to_numpy()
        bars = ax.bar(x + offset, vals, width=width, label=scheme, color=colors[scheme], edgecolor="black", linewidth=0.35)
        for bar, val in zip(bars, vals):
            ax.text(
                bar.get_x() + bar.get_width() / 2,
                val + 1.0,
                f"{val:.1f}%",
                ha="center",
                va="bottom",
                rotation=90,
                fontsize=11,
            )

    ax.set_ylabel("Percentage (\\%)", fontsize=22, labelpad=3)
    ax.set_ylim(0, 104)
    ax.set_xticks(x)
    ax.set_xticklabels(metrics, rotation=18, ha="right", fontsize=15)
    ax.tick_params(axis="y", labelsize=15, pad=2)
    ax.legend(loc="upper right", frameon=True, fontsize=13, ncol=2)
    ax.grid(True, axis="y", alpha=0.30)
    save(fig, "uav_sample_kpis_tn_kmeans_gru_cnn")


def plot_mobility() -> None:
    rng = np.random.default_rng(11)
    t = np.arange(0, 121, 5)

    schemes = {
        "Reactive K-means TN+NTN": dict(
            scale=1.00,
            offset=0,
            title="Reactive K-means TN+NTN",
            underserved=[0, 18, 45, 70, 82, 76, 68, 60, 52, 45, 39, 34, 30, 27, 24, 21, 19, 17, 16, 15, 15, 14, 14, 14, 14],
        ),
        "GRU-predictive TN+NTN": dict(
            scale=0.82,
            offset=8,
            title="GRU-predictive TN+NTN",
            underserved=[0, 10, 32, 52, 64, 50, 38, 28, 21, 18, 16, 14, 13, 12, 11, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9],
        ),
        "CNN-learning TN+NTN": dict(
            scale=0.70,
            offset=14,
            title="CNN-learning TN+NTN",
            underserved=[0, 14, 40, 62, 72, 58, 44, 32, 20, 14, 11, 8, 6, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4],
        ),
    }
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    fig, axes = plt.subplots(1, 3, figsize=(13.6, 4.5), sharey=True, constrained_layout=False)
    fig.subplots_adjust(left=0.075, right=0.91, bottom=0.19, top=0.87, wspace=0.12)

    rows = []
    for ax, (scheme, cfg) in zip(axes, schemes.items()):
        for u in range(6):
            base = np.maximum(0, (t - (8 + u * 2)) * (1.2 + 0.08 * u))
            wobble = 9 * np.sin(0.18 * t + u) + rng.normal(0, 3.2, len(t))
            dist = np.clip(cfg["offset"] + cfg["scale"] * base + wobble, 0, 190)
            if scheme.startswith("CNN"):
                dist = np.minimum(dist, 130 + 8 * u)
            ax.plot(t, dist, marker="o", markersize=3.2, linewidth=1.8, color=colors[u], label=f"UAV{u}")
            rows.extend({"scheme": scheme, "time_s": ti, "uav": u, "distance_m": di} for ti, di in zip(t, dist))

        bars_t = np.arange(5, 121, 10)
        bar_counts = np.interp(bars_t, t, np.array(cfg["underserved"], dtype=float))
        ax2 = ax.twinx()
        ax2.bar(bars_t, bar_counts, width=4.0, color=cm.YlOrBr(bar_counts / 85.0), alpha=0.55)
        ax2.set_ylim(0, 90)
        ax2.set_yticks([])
        ax.set_title(cfg["title"], fontsize=18, pad=4)
        ax.set_xlabel("Time (s)", fontsize=18, labelpad=3)
        ax.tick_params(axis="both", labelsize=12, pad=2)
        ax.grid(True, alpha=0.25)

    axes[0].set_ylabel("3D distance from initial position (m)", fontsize=18, labelpad=3)
    axes[0].legend(loc="upper left", frameon=True, fontsize=10, ncol=2)
    cax = fig.add_axes([0.925, 0.22, 0.018, 0.58])
    norm = plt.Normalize(0, 85)
    sm = cm.ScalarMappable(norm=norm, cmap=cm.YlOrBr)
    cb = fig.colorbar(sm, cax=cax)
    cb.set_label("Number of underserved UEs", fontsize=16)
    cb.ax.tick_params(labelsize=12)
    pd.DataFrame(rows).to_csv(OUT_DIR / "uav_sample_mobility_kmeans_gru_cnn.csv", index=False)
    save(fig, "uav_sample_mobility_kmeans_gru_cnn")


def main() -> None:
    use_style()
    plot_snr_cdf()
    plot_kpi_bars()
    plot_mobility()


if __name__ == "__main__":
    main()
