#!/usr/bin/env python3
"""Plot the RF workflow's reduction in expensive ns-3 invocations."""

from pathlib import Path

import matplotlib.pyplot as plt


def main() -> None:
    output_dir = Path("results/uav-isac-drl/rf-paper-v1/paper-results")
    output_dir.mkdir(parents=True, exist_ok=True)

    exhaustive_runs = 100_000
    design_runs = 300
    paired_test_runs = 2 * 20
    rf_workflow_runs = design_runs + paired_test_runs
    reduction_pct = 100.0 * (1.0 - rf_workflow_runs / exhaustive_runs)
    reduction_factor = exhaustive_runs / rf_workflow_runs

    plt.rcParams.update({
        "font.family": "serif",
        "font.size": 8,
        "axes.titlesize": 9,
        "axes.labelsize": 8,
    })

    fig, ax = plt.subplots(figsize=(3.5, 3.0))
    fig.subplots_adjust(left=0.22, right=0.98, top=0.86, bottom=0.25)
    bars = ax.bar(
        ["Exhaustive ns-3\nscreening", "RF-assisted\nworkflow"],
        [exhaustive_runs, rf_workflow_runs],
        color=["#8c8c8c", "#2878b5"],
        edgecolor="black",
        linewidth=0.6,
        width=0.58,
    )
    ax.set_yscale("log")
    ax.set_ylim(100, 300_000)
    ax.set_ylabel("Number of ns-3 runs (log scale)")
    ax.set_title("Reduction in Expensive ns-3 Invocations")
    ax.grid(axis="y", which="both", linestyle=":", linewidth=0.5, alpha=0.65)

    ax.bar_label(
        bars,
        labels=["100,000", "340"],
        padding=3,
        fontsize=8,
        fontweight="bold",
    )
    ax.text(
        1,
        125,
        "300 design\n+ 40 paired tests",
        ha="center",
        va="bottom",
        fontsize=7,
    )
    ax.text(
        0.5,
        0.76,
        f"{reduction_pct:.2f}% fewer ns-3 calls\n({reduction_factor:.0f}× reduction)",
        transform=ax.transAxes,
        ha="center",
        va="center",
        fontsize=8,
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "edgecolor": "#2878b5"},
    )
    for suffix in ("png", "pdf"):
        fig.savefig(
            output_dir / f"rf_computational_reduction.{suffix}",
            dpi=300,
            bbox_inches="tight",
        )
    plt.close(fig)

    print(f"Exhaustive screening calls: {exhaustive_runs}")
    print(f"RF-assisted workflow calls: {rf_workflow_runs}")
    print(f"Reduction: {reduction_pct:.2f}% ({reduction_factor:.1f}x fewer calls)")
    print(f"Wrote figures to {output_dir}")


if __name__ == "__main__":
    main()
