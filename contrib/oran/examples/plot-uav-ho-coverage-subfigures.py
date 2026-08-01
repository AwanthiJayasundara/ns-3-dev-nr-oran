#!/usr/bin/env python3
"""Generate separate HO/backhaul/coverage panels for LaTeX subfigures."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

from importlib.machinery import SourceFileLoader


SCRIPT = Path(__file__).resolve().parent / "plot-uav-ho-success-reject-ai-comparison.py"
mod = SourceFileLoader("ho_kpi", str(SCRIPT)).load_module()

DATASET_ROOT = Path("results/nr/tn-ntn/ai-dataset-v2")
AI_RUN = Path(
    "results/nr/tn-ntn/"
    "tn-uav-satellite_ueS1_50_ueS2_50_tnGnb_4_ntnGnb_3_tnCap_20_ntnCap_10_hyst_2_ai-switching-sat"
)
OUT_DIR = Path("docs/figures")
NUM_TN_CELLS = 4
COVERAGE_THRESHOLD_DBM = -110.0


def setup_style() -> None:
    style_path = Path("./latex_style.mplstyle")
    if not style_path.exists():
        style_path = Path(__file__).resolve().parents[3] / "latex_style.mplstyle"
    if style_path.exists():
        plt.style.use(style_path)
    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "font.size": 13,
            "axes.labelsize": 14,
            "xtick.labelsize": 12,
            "ytick.labelsize": 12,
            "legend.fontsize": 10,
        }
    )


def save(fig: plt.Figure, name: str) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_DIR / f"{name}.pdf")
    fig.savefig(OUT_DIR / f"{name}.png", dpi=300)
    print(f"[saved] {OUT_DIR / f'{name}.pdf'}")
    print(f"[saved] {OUT_DIR / f'{name}.png'}")


def main() -> None:
    setup_style()

    data = mod.collect(DATASET_ROOT, AI_RUN, NUM_TN_CELLS)
    summary = (
        data.groupby("Controller", as_index=False)
        .agg(
            TnSuccess=("TnSuccess", "mean"),
            NtnSuccess=("NtnSuccess", "mean"),
            TnLowRsrpReject=("TnLowRsrpReject", "mean"),
            NtnLowRsrpReject=("NtnLowRsrpReject", "mean"),
            Runs=("Controller", "size"),
        )
        .sort_values("Controller", key=lambda s: s.map({"Rule-based": 0, "RF-AI": 1}))
    )
    labels = summary["Controller"].tolist()
    x = range(len(summary))
    tn_color = "#4C78A8"
    ntn_color = "#F28E2B"

    fig, ax = plt.subplots(figsize=(4.6, 3.45), constrained_layout=True)
    ax.bar(x, summary["TnSuccess"], color=tn_color, label="TN target")
    ax.bar(x, summary["NtnSuccess"], bottom=summary["TnSuccess"], color=ntn_color, label="UAV/NTN target")
    ax.set_ylabel("Count")
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels)
    ax.grid(True, axis="y", linestyle="--", alpha=0.38)
    ax.legend(frameon=True)
    save(fig, "uav_subfig_ho_success")
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(4.6, 3.45), constrained_layout=True)
    ax.bar(x, summary["TnLowRsrpReject"], color=tn_color, label="TN candidate")
    ax.bar(
        x,
        summary["NtnLowRsrpReject"],
        bottom=summary["TnLowRsrpReject"],
        color=ntn_color,
        label="UAV/NTN candidate",
    )
    ax.set_ylabel("Count")
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels)
    ax.grid(True, axis="y", linestyle="--", alpha=0.38)
    ax.legend(frameon=True)
    save(fig, "uav_subfig_ric_low_rsrp_rejects")
    plt.close(fig)

    donor_by_uav = mod.collect_donor_by_uav(DATASET_ROOT, AI_RUN, NUM_TN_CELLS)
    donor_by_uav["BarLabel"] = donor_by_uav.apply(
        lambda row: ("Rule" if row["Controller"] == "Rule-based" else "RF-AI")
        + f"\nUAV {int(row['UavIndex'])}",
        axis=1,
    )
    donor_cols = [f"DonorCell{cell_id}" for cell_id in range(1, NUM_TN_CELLS + 1)] + [
        "SatelliteFallback"
    ]
    donor_labels = [f"TN donor {cell_id}" for cell_id in range(1, NUM_TN_CELLS + 1)] + [
        "Satellite fallback"
    ]
    donor_colors = ["#76B7B2", "#EDC948", "#B07AA1", "#9C755F", "#F28E2B"]
    fig, ax = plt.subplots(figsize=(5.8, 3.45), constrained_layout=True)
    donor_x = range(len(donor_by_uav))
    bottom = pd.Series([0.0] * len(donor_by_uav))
    for col, label, color in zip(donor_cols, donor_labels, donor_colors):
        ax.bar(donor_x, donor_by_uav[col], bottom=bottom, color=color, label=label)
        bottom = bottom + donor_by_uav[col].reset_index(drop=True)
    ax.set_ylabel("Trace samples")
    ax.set_xticks(list(donor_x))
    ax.set_xticklabels(donor_by_uav["BarLabel"], fontsize=9)
    ax.grid(True, axis="y", linestyle="--", alpha=0.38)
    ax.legend(frameon=True, fontsize=8, ncol=1, loc="upper left")
    save(fig, "uav_subfig_backhaul_by_uav")
    plt.close(fig)

    coverage = mod.collect_coverage(DATASET_ROOT, AI_RUN, COVERAGE_THRESHOLD_DBM)
    coverage_mean = coverage.groupby("Controller")["coverage_pct"].mean().to_dict()
    fig, ax = plt.subplots(figsize=(4.8, 3.45), constrained_layout=True)
    styles = {
        "Rule-based": ("#F28E2B", "o", "-."),
        "RF-AI": ("#4C78A8", "D", "-"),
    }
    for controller, (color, marker, linestyle) in styles.items():
        part = coverage[coverage["Controller"].eq(controller)]
        if part.empty:
            continue
        label = f"{controller} ({coverage_mean.get(controller, 0.0):.1f}\\%)"
        ax.plot(
            part["time"],
            part["coverage_pct"],
            color=color,
            marker=marker,
            markevery=max(1, len(part) // 5),
            linestyle=linestyle,
            linewidth=1.9,
            label=label,
        )
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Coverage (%)")
    ax.set_xlim(0, 120)
    ax.set_ylim(0, 100)
    ax.grid(True, linestyle="--", alpha=0.38)
    ax.legend(frameon=True, loc="lower right")
    save(fig, "uav_subfig_serving_coverage")
    plt.close(fig)


if __name__ == "__main__":
    main()
