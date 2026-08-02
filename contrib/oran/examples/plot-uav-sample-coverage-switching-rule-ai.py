#!/usr/bin/env python3
"""Create a sample coverage/switching figure for the clustered UAV scenario.

This plot is an expected/target interpretation figure. It is not produced from
measured ns-3 packet counters. Use it to illustrate the intended comparison
before replacing it with measured coverage after the clustered scenario is run.
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


OUT_DIR = Path("docs/figures")
OUT_DIR.mkdir(parents=True, exist_ok=True)


def interp_series(points, t_grid):
    x, y = zip(*points)
    return np.interp(t_grid, np.array(x, dtype=float), np.array(y, dtype=float))


def main():
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Times New Roman", "DejaVu Serif", "Times"],
            "mathtext.fontset": "dejavuserif",
            "axes.labelsize": 18,
            "xtick.labelsize": 15,
            "ytick.labelsize": 15,
            "legend.fontsize": 14,
            "axes.linewidth": 1.4,
            "lines.linewidth": 2.8,
            "figure.dpi": 180,
            "savefig.dpi": 300,
        }
    )

    t = np.arange(0, 201, 5)

    # Underserved coverage for three demand levels. With 3 UAVs and capacity
    # 10 each, 20 UEs is light demand, 30 UEs is capacity-matched, and 40 UEs
    # is overloaded. Faster convergence to target coverage in each demand case.
    series = {
        "Rule, 20 UEs": (
            "#D64B4B",
            "-",
            "s",
            [(0, 0), (10, 18), (30, 45), (50, 66), (70, 82), (90, 94), (130, 96), (200, 95)],
        ),
        "Rule, 30 UEs": (
            "#D64B4B",
            "--",
            "s",
            [(0, 0), (10, 12), (30, 30), (50, 48), (70, 66), (90, 80), (115, 91), (160, 93), (200, 92)],
        ),
        "Rule, 40 UEs": (
            "#D64B4B",
            ":",
            "s",
            [(0, 0), (10, 9), (30, 24), (50, 38), (70, 52), (90, 65), (120, 76), (160, 82), (200, 80)],
        ),
        "RF-AI, 20 UEs": (
            "#2E6EB5",
            "-",
            "D",
            [(0, 0), (10, 28), (30, 62), (45, 82), (60, 94), (90, 98), (160, 97), (200, 96)],
        ),
        "RF-AI, 30 UEs": (
            "#2E6EB5",
            "--",
            "D",
            [(0, 0), (10, 22), (30, 50), (50, 76), (65, 88), (75, 93), (120, 96), (200, 94)],
        ),
        "RF-AI, 40 UEs": (
            "#2E6EB5",
            ":",
            "D",
            [(0, 0), (10, 18), (30, 42), (50, 64), (70, 78), (90, 86), (130, 90), (200, 88)],
        ),
    }

    fig, ax_cov = plt.subplots(figsize=(10.8, 5.2))

    for label, (color, linestyle, marker, points) in series.items():
        ax_cov.plot(
            t,
            interp_series(points, t),
            color=color,
            linestyle=linestyle,
            marker=marker,
            markevery=4,
            label=label,
        )
    ax_cov.axhline(90, color="#444444", linestyle=":", linewidth=2.0, label="90% target")
    ax_cov.fill_between(t, 90, 100, color="#7BC87C", alpha=0.10)
    ax_cov.set_ylabel("Underserved UE coverage (%)", labelpad=8)
    ax_cov.set_ylim(0, 105)
    ax_cov.set_xlim(0, 200)
    ax_cov.grid(True, linestyle="--", alpha=0.38)
    ax_cov.legend(loc="lower right", ncol=2, frameon=True, fontsize=12)

    ax_cov.annotate(
        "Faster convergence to target coverage",
        xy=(75, 93),
        xytext=(86, 72),
        arrowprops=dict(arrowstyle="->", lw=1.5, color="#2E6EB5"),
        color="#2E6EB5",
        fontsize=15,
    )
    ax_cov.annotate(
        "Rule-based improves later",
        xy=(115, 91),
        xytext=(124, 51),
        arrowprops=dict(arrowstyle="->", lw=1.5, color="#D64B4B"),
        color="#D64B4B",
        fontsize=15,
    )

    ax_cov.axvspan(35, 62, color="#2E6EB5", alpha=0.06)
    ax_cov.axvspan(58, 96, color="#D64B4B", alpha=0.06)
    ax_cov.set_xlabel("Simulation time (s)")

    fig.subplots_adjust(left=0.15, right=0.985, bottom=0.18, top=0.98)
    png = OUT_DIR / "uav_sample_underserved_coverage_switching_rule_ai.png"
    pdf = OUT_DIR / "uav_sample_underserved_coverage_switching_rule_ai.pdf"
    fig.savefig(png, bbox_inches="tight")
    fig.savefig(pdf, bbox_inches="tight")
    print(f"[saved] {png}")
    print(f"[saved] {pdf}")


if __name__ == "__main__":
    main()
