#!/usr/bin/env python3
"""Plot RF-AI switching decisions against donor-link RSRP per UAV."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.lines import Line2D


DEFAULT_RUN = Path(
    "results/nr/tn-ntn/"
    "tn-uav-satellite_ueS1_50_ueS2_50_tnGnb_4_ntnGnb_3_tnCap_20_ntnCap_10_hyst_2_ai-switching-sat"
)

MODE_STYLE = {
    "TN_DIRECT": ("#59A14F", "o", "TN direct"),
    "SATELLITE_FALLBACK": ("#F28E2B", "s", "Satellite fallback"),
    "NO_BACKHAUL_AVAILABLE": ("#E15759", "X", "No backhaul"),
}


def load_style() -> None:
    style_path = Path("./latex_style.mplstyle")
    if not style_path.exists():
        style_path = Path(__file__).resolve().parents[3] / "latex_style.mplstyle"
    if style_path.exists():
        plt.style.use(style_path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--threshold-dbm", type=float, default=-100.0)
    parser.add_argument("--delay-label", type=str, default="Switching delay")
    parser.add_argument("--output", type=Path, default=Path("docs/figures/uav_rf_ai_decision_rsrp.pdf"))
    parser.add_argument("--png", type=Path, default=Path("docs/figures/uav_rf_ai_decision_rsrp.png"))
    parser.add_argument("--csv", type=Path, default=Path("docs/figures/uav_rf_ai_decision_rsrp_summary.csv"))
    args = parser.parse_args()

    trace = args.run_dir / "xhaul-autonomy-trace.csv"
    if not trace.exists():
        raise FileNotFoundError(trace)

    load_style()
    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "font.size": 17,
            "axes.titlesize": 20,
            "axes.labelsize": 24,
            "xtick.labelsize": 20,
            "ytick.labelsize": 20,
            "legend.fontsize": 16,
            "lines.markersize": 6,
        }
    )

    df = pd.read_csv(trace).sort_values(["UavIndex", "Time"])
    uav_indices = sorted(df["UavIndex"].unique())
    fig, axes = plt.subplots(
        len(uav_indices),
        1,
        figsize=(8.8, 7.0),
        sharex=True,
        constrained_layout=False,
    )
    if len(uav_indices) == 1:
        axes = [axes]

    summary_rows = []
    for ax, uav_idx in zip(axes, uav_indices):
        group = df[df["UavIndex"].eq(uav_idx)]
        ax.plot(
            group["Time"],
            group["XhaulRsrpDbm"],
            color="#4C78A8",
            linewidth=2.0,
            label="Donor-link RSRP",
        )
        ax.axhline(
            args.threshold_dbm,
            color="#C44E52",
            linestyle=":",
            linewidth=1.6,
            label=f"{args.threshold_dbm:.0f} dBm threshold",
        )

        marker_y = group["XhaulRsrpDbm"].max() + 2.0
        for mode, (color, marker, label) in MODE_STYLE.items():
            part = group[group["BackhaulMode"].eq(mode)]
            if part.empty:
                continue
            ax.scatter(
                part["Time"],
                [marker_y] * len(part),
                color=color,
                marker=marker,
                s=42 if marker != "X" else 56,
                edgecolors="black" if marker == "X" else "none",
                linewidths=0.35,
                label=label,
                zorder=4,
            )

        ax.set_title(f"UAV {int(uav_idx)}", loc="left", pad=4)
        ax.grid(True, linestyle="--", alpha=0.38)
        ax.set_ylim(group["XhaulRsrpDbm"].min() - 5.0, marker_y + 4.0)

        first_below = group[group["XhaulRsrpDbm"].lt(args.threshold_dbm)]["Time"].min()
        first_sat = group[group["BackhaulMode"].eq("SATELLITE_FALLBACK")]["Time"].min()
        summary_rows.append(
            {
                "UavIndex": int(uav_idx),
                "FirstBelowThresholdS": first_below,
                "FirstSatelliteFallbackS": first_sat,
                "SwitchingDelayS": None
                if pd.isna(first_below) or pd.isna(first_sat)
                else float(first_sat - first_below),
                "TnDirectCount": int(group["BackhaulMode"].eq("TN_DIRECT").sum()),
                "SatelliteFallbackCount": int(group["BackhaulMode"].eq("SATELLITE_FALLBACK").sum()),
                "NoBackhaulCount": int(group["BackhaulMode"].eq("NO_BACKHAUL_AVAILABLE").sum()),
            }
        )

    axes[-1].set_xlabel("UAV flight time (s)")
    axes[-1].set_xlim(0, 120)
    fig.supylabel("TN donor-link RSRP to UAV (dBm)", x=0.04, fontsize=22)
    fig.subplots_adjust(left=0.17, right=0.985, top=0.92, bottom=0.13, hspace=0.40)

    handles = [
        Line2D([0], [0], color="#4C78A8", linewidth=2.0, label="Donor-link RSRP"),
        Line2D([0], [0], color="#C44E52", linestyle=":", linewidth=1.6, label="RSRP threshold"),
    ]
    handles.extend(
        Line2D(
            [0],
            [0],
            marker=marker,
            color="none",
            markerfacecolor=color,
            markeredgecolor="black" if marker == "X" else color,
            linestyle="none",
            markersize=8,
            label=label,
        )
        for _, (color, marker, label) in MODE_STYLE.items()
    )
    axes[0].legend(handles=handles, loc="upper right", frameon=True, ncol=2)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output)
    fig.savefig(args.png, dpi=300)
    pd.DataFrame(summary_rows).to_csv(args.csv, index=False)

    print(f"[saved] {args.output}")
    print(f"[saved] {args.png}")
    print(f"[saved] {args.csv}")
    for row in summary_rows:
        print(
            f"[summary] UAV {row['UavIndex']}: first SAT={row['FirstSatelliteFallbackS']}, "
            f"TN={row['TnDirectCount']}, SAT={row['SatelliteFallbackCount']}, "
            f"NO={row['NoBackhaulCount']}"
        )


if __name__ == "__main__":
    main()
