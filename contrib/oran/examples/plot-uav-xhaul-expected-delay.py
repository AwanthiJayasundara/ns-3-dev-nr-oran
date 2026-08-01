#!/usr/bin/env python3
"""Generate an expected/target delay figure for UAV TN/NTN fallback."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt


def make_series() -> tuple[list[int], dict[str, list[float]]]:
    times = list(range(0, 121, 5))
    series = {
        "TN + UAV, no satellite": [],
        "TN + UAV + satellite": [],
        "TN + UAV + satellite + AI": [],
    }

    for t in times:
        # Normal TN wireless-backhaul delay before/after outage.
        normal = 28.0 + 3.0 * ((t // 15) % 2)

        # No satellite: during backhaul outage, delivered traffic stalls. A
        # timeout-like delay is used to visualize the service-continuity penalty.
        if 30 <= t <= 75:
            series["TN + UAV, no satellite"].append(720.0 + 35.0 * ((t // 10) % 2))
        else:
            series["TN + UAV, no satellite"].append(normal)

        # Satellite fallback: higher than TN_DIRECT because of the
        # UAV -> SAT -> GW -> Core path, but much lower than no-service timeout.
        if 30 <= t <= 75:
            series["TN + UAV + satellite"].append(255.0 + 18.0 * ((t // 10) % 2))
        else:
            series["TN + UAV + satellite"].append(normal + 2.0)

        # AI switching: expected to reduce switching latency/instability during
        # fallback while still using the same longer satellite path.
        if 30 <= t <= 75:
            series["TN + UAV + satellite + AI"].append(225.0 + 12.0 * ((t // 10) % 2))
        else:
            series["TN + UAV + satellite + AI"].append(normal + 1.0)

    return times, series


def main() -> None:
    out_dir = Path("docs/figures")
    out_dir.mkdir(parents=True, exist_ok=True)
    prefix = "expected_uav_xhaul_delay_over_time"

    style_path = Path("./latex_style.mplstyle")
    if not style_path.exists():
        style_path = Path(__file__).resolve().parents[3] / "latex_style.mplstyle"
    if style_path.exists():
        plt.style.use(style_path)

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

    times, series = make_series()
    styles = {
        "TN + UAV, no satellite": ("#E15759", "s", "--"),
        "TN + UAV + satellite": ("#F28E2B", "o", "-."),
        "TN + UAV + satellite + AI": ("#4C78A8", "D", "-"),
    }

    fig, ax = plt.subplots(figsize=(7.8, 4.8), constrained_layout=True)
    for label, values in series.items():
        color, marker, linestyle = styles[label]
        ax.plot(
            times,
            values,
            label=label,
            color=color,
            marker=marker,
            linestyle=linestyle,
            linewidth=2.4,
        )

    ax.axvspan(30, 75, color="#C44E52", alpha=0.10)
    ax.axvline(30, color="#C44E52", linestyle=":", linewidth=1.4)
    ax.axvline(75, color="#C44E52", linestyle=":", linewidth=1.4)
    ax.text(
        52.5,
        620,
        "TN wireless backhaul unavailable",
        ha="center",
        va="center",
        color="#8B2E2F",
        fontsize=16,
    )

    ax.set_xlabel("Simulation time (s)")
    ax.set_ylabel("DL delay (ms)")
    ax.set_xlim(0, 120)
    ax.set_ylim(0, 820)
    ax.grid(True, linestyle="--", alpha=0.45)
    ax.legend(loc="upper right", frameon=True)

    png = out_dir / f"{prefix}.png"
    pdf = out_dir / f"{prefix}.pdf"
    csv_path = out_dir / f"{prefix}.csv"
    tex = out_dir / f"{prefix}.tex"
    fig.savefig(png, dpi=300)
    fig.savefig(pdf)

    with csv_path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["TimeS", *series.keys()])
        for i, t in enumerate(times):
            writer.writerow([t, *[series[label][i] for label in series]])

    tex.write_text(
        "\\begin{figure}[!t]\n"
        "  \\centering\n"
        f"  \\includegraphics[width=0.92\\linewidth]{{figures/{pdf.name}}}\n"
        "  \\caption{Expected downlink delay behavior under TN wireless backhaul outage. "
        "Satellite fallback increases delay compared with direct TN backhaul but avoids the no-service timeout behavior.}\n"
        "  \\label{fig:expected-uav-xhaul-delay}\n"
        "\\end{figure}\n",
        encoding="utf-8",
    )

    print(f"[saved] {png}")
    print(f"[saved] {pdf}")
    print(f"[saved] {csv_path}")
    print(f"[saved] {tex}")


if __name__ == "__main__":
    main()
