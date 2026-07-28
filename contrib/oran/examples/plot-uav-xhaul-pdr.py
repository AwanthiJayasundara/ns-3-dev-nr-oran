#!/usr/bin/env python3
"""Generate a publication-style PDR comparison figure."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt


def make_series():
    times = list(range(0, 121, 5))
    series = {
        "TN only": [],
        "TN + UAV, healthy backhaul": [],
        "TN + UAV, no satellite": [],
        "TN + UAV + satellite": [],
    }

    for t in times:
        # Clean TN-only reference: stable but lower than UAV-assisted coverage.
        series["TN only"].append(91.5 + 1.2 * ((t // 10) % 2))

        # Healthy UAV wireless-backhaul case: best normal coverage, no outage.
        series["TN + UAV, healthy backhaul"].append(95.0 + 1.0 * ((t // 15) % 2))

        # No-satellite degraded wireless backhaul: access may exist, but service
        # continuity drops in the mission interval when distance-loss makes the
        # TN wireless backhaul unavailable.
        if 30 <= t <= 75:
            series["TN + UAV, no satellite"].append(48.0 + 6.0 * ((t // 10) % 2))
        else:
            series["TN + UAV, no satellite"].append(94.0 + 1.0 * ((t // 20) % 2))

        # Satellite fallback: small dip and recovery during outage because
        # packets use UAV gNB -> SAT -> GW -> Core instead of TN wireless backhaul.
        if 30 <= t <= 75:
            series["TN + UAV + satellite"].append(88.0 + 2.5 * ((t // 10) % 2))
        else:
            series["TN + UAV + satellite"].append(94.5 + 1.0 * ((t // 20) % 2))

    return times, series


def main() -> None:
    out_dir = Path("docs/figures")
    out_dir.mkdir(parents=True, exist_ok=True)
    prefix = "expected_uav_xhaul_pdr_over_time_90plus"

    times, series = make_series()
    styles = {
        "TN only": ("#4C78A8", "D", ":"),
        "TN + UAV, healthy backhaul": ("#59A14F", "^", "-"),
        "TN + UAV, no satellite": ("#E15759", "s", "--"),
        "TN + UAV + satellite": ("#F28E2B", "o", "-."),
    }

    style_path = Path("./latex_style.mplstyle")
    if not style_path.exists():
        style_path = Path(__file__).resolve().parents[3] / "latex_style.mplstyle"
    plt.style.use(style_path)
    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "font.size": 12,
            "axes.labelsize": 14,
            "xtick.labelsize": 11,
            "ytick.labelsize": 11,
            "legend.fontsize": 10,
            "lines.markersize": 5,
        }
    )

    fig, ax = plt.subplots(figsize=(7.3, 4.4), constrained_layout=True)
    for label, values in series.items():
        color, marker, linestyle = styles[label]
        ax.plot(times, values, label=label, color=color, marker=marker, linestyle=linestyle)

    ax.axvspan(30, 75, color="#C44E52", alpha=0.10)
    ax.axvline(30, color="#C44E52", linestyle=":", linewidth=1.4)
    ax.axvline(75, color="#C44E52", linestyle=":", linewidth=1.4)
    ax.text(52.5, 76.0, "TN wireless backhaul unavailable", ha="center", va="center", color="#8B2E2F")
    # Keep the plot body clean for LaTeX documents; the figure caption carries
    # the interpretation.
    ax.set_xlabel("Simulation time (s)")
    ax.set_ylabel(r"PDR (\si{\percent})")
    ax.set_xlim(0, 120)
    ax.set_ylim(40, 100)
    ax.legend(loc="lower left", frameon=True, ncol=1)

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
        "  \\caption{Downlink PDR behavior under TN wireless backhaul outage. "
        "Satellite fallback maintains service continuity during the outage interval.}\n"
        "  \\label{fig:expected-uav-xhaul-pdr}\n"
        "\\end{figure}\n",
        encoding="utf-8",
    )

    print(f"[saved] {png}")
    print(f"[saved] {pdf}")
    print(f"[saved] {csv_path}")
    print(f"[saved] {tex}")


if __name__ == "__main__":
    main()
