#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

DEFAULT_INPUT = Path(
    "results/nr/tn-ntn/ueS1_120_ueS2_120_tnGnb_8_ntnCap_10_hyst_2/ml-ho-dataset.csv"
)

# corrected path from your run folder name
if not DEFAULT_INPUT.exists():
    DEFAULT_INPUT = Path(
        "results/nr/tn-ntn/ueS1_120_ueS2_120_tnGnb_8_ntnGnb_6_tnCap_20_ntnCap_10_hyst_2/ml-ho-dataset.csv"
    )

DEFAULT_OUTDIR = Path("results/nr/tn-ntn/plots")
DEFAULT_STYLE = Path("./latex_style.mplstyle")


def apply_plot_style() -> None:
    if DEFAULT_STYLE.exists():
        plt.style.use(str(DEFAULT_STYLE))

    plt.rcParams.update({
        "font.size": 20,
        "axes.titlesize": 20,
        "axes.labelsize": 20,
        "xtick.labelsize": 20,
        "ytick.labelsize": 20,
        "legend.fontsize": 20,
    })


def build_summary_from_raw(
    csv_path: Path,
    rsrp_min: float = -120.0,
    t_max: float = 30.0,
) -> pd.DataFrame:
    df = pd.read_csv(csv_path)

    needed = {
        "time",
        "ueId",
        "candidateRsrp",
        "candidateEffLoad",
        "candidateCap",
        "candidateIsNtn",
    }
    missing = needed - set(df.columns)
    if missing:
        raise ValueError(f"Missing required columns: {sorted(missing)}")

    df = df[df["time"] <= t_max].copy()

    # basic feasibility
    df["radio_ok"] = df["candidateRsrp"] >= rsrp_min
    df["load_ok"] = df["candidateEffLoad"] < df["candidateCap"]

    # TN-only coverage: only terrestrial candidates
    df["tn_feasible"] = (
        (df["candidateIsNtn"] == 0)
        & df["radio_ok"]
        & df["load_ok"]
    )

    # TN-NTN coverage: either TN or NTN candidate is feasible
    df["tn_ntn_feasible"] = df["radio_ok"] & df["load_ok"]

    # one row per (time, ueId)
    per_ue = df.groupby(["time", "ueId"], as_index=False).agg(
        tn_only_covered=("tn_feasible", "max"),
        tn_ntn_covered=("tn_ntn_feasible", "max"),
    )

    total_ues = per_ue.groupby("time")["ueId"].nunique().rename("total_ues")
    covered = per_ue.groupby("time").agg(
        tn_only_covered=("tn_only_covered", "sum"),
        tn_ntn_covered=("tn_ntn_covered", "sum"),
    )

    summary = covered.join(total_ues).reset_index().sort_values("time")
    summary["tn_only_cov_pct"] = 100.0 * summary["tn_only_covered"] / summary["total_ues"]
    summary["tn_ntn_cov_pct"] = 100.0 * summary["tn_ntn_covered"] / summary["total_ues"]
    summary["improvement_pct_points"] = summary["tn_ntn_cov_pct"] - summary["tn_only_cov_pct"]

    return summary


def print_terminal_summary(summary: pd.DataFrame) -> None:
    mean_tn = summary["tn_only_cov_pct"].mean()
    mean_tnntn = summary["tn_ntn_cov_pct"].mean()
    mean_gain = summary["improvement_pct_points"].mean()

    max_gain_row = summary.loc[summary["improvement_pct_points"].idxmax()]
    min_gain_row = summary.loc[summary["improvement_pct_points"].idxmin()]

    print("\n===== COVERAGE SUMMARY =====")
    print(f"Average TN-only coverage           : {mean_tn:.2f}%")
    print(f"Average TN+NTN coverage            : {mean_tnntn:.2f}%")
    print(f"Average improvement                : {mean_gain:.2f} percentage points")
    print(f"Maximum improvement                : {max_gain_row['improvement_pct_points']:.2f} percentage points at t={max_gain_row['time']:.2f}s")
    print(f"Minimum improvement                : {min_gain_row['improvement_pct_points']:.2f} percentage points at t={min_gain_row['time']:.2f}s")

    print("\nPer-time coverage values:")
    for _, row in summary.iterrows():
        print(
            f"t={row['time']:.2f}s | "
            f"TN={row['tn_only_cov_pct']:.2f}% | "
            f"TN+NTN={row['tn_ntn_cov_pct']:.2f}% | "
            f"Gain={row['improvement_pct_points']:.2f} pp"
        )


def make_plot(summary: pd.DataFrame, out_png: Path) -> None:
    apply_plot_style()

    x = summary["time"]
    y_tn = summary["tn_only_cov_pct"]
    y_tnntn = summary["tn_ntn_cov_pct"]

    ymin = min(y_tn.min(), y_tnntn.min())
    ymax = max(y_tn.max(), y_tnntn.max())
    pad = max(1.0, 0.15 * (ymax - ymin))

    fig, ax = plt.subplots(figsize=(8.5, 5.2))

    ax.fill_between(
        x,
        y_tn,
        y_tnntn,
        alpha=0.25,
        label="Coverage gain of TN+NTN",
    )

    ax.plot(
        x,
        y_tn,
        marker="o",
        linewidth=2.4,
        markersize=5,
        label="TN-only",
    )

    ax.plot(
        x,
        y_tnntn,
        marker="s",
        linewidth=2.4,
        markersize=5,
        color="tab:green",
        label="TN+NTN",
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Feasible coverage ratio (\%)")
    ax.set_ylim(max(0, ymin - pad), min(100, ymax + 0.5))
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", frameon=True)

    fig.tight_layout()
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=300, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Build TN-only vs TN+NTN summary from raw ml-ho-dataset.csv and plot it."
    )
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--outdir", type=Path, default=DEFAULT_OUTDIR)
    parser.add_argument("--rsrp-min", type=float, default=-120.0)
    parser.add_argument("--t-max", type=float, default=50.0)
    args = parser.parse_args()

    summary = build_summary_from_raw(
        csv_path=args.input,
        rsrp_min=args.rsrp_min,
        t_max=args.t_max,
    )

    args.outdir.mkdir(parents=True, exist_ok=True)

    summary_csv = args.outdir / "tn_vs_tnntn_summary.csv"
    out_png = args.outdir / "fig_tn_vs_tnntn_shaded.png"

    summary.to_csv(summary_csv, index=False)
    make_plot(summary, out_png)
    print_terminal_summary(summary)

    print(f"\nSaved summary CSV: {summary_csv}")
    print(f"Saved figure: {out_png}")


if __name__ == "__main__":
    main()