#!/usr/bin/env python3
"""Plot UAV 3D mobility with RF-AI switching and donor-link RSRP evidence."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


DEFAULT_RUN = Path(
    "results/nr/tn-ntn/"
    "tn-uav-satellite_ueS1_50_ueS2_50_tnGnb_4_ntnGnb_3_tnCap_20_ntnCap_10_hyst_2_ai-switching-sat"
)

MODE_COLORS = {
    "TN_DIRECT": "#59A14F",
    "SATELLITE_FALLBACK": "#F28E2B",
    "NO_BACKHAUL_AVAILABLE": "#E15759",
}

MODE_MARKERS = {
    "TN_DIRECT": "o",
    "SATELLITE_FALLBACK": "s",
    "NO_BACKHAUL_AVAILABLE": "X",
}


def load_style() -> None:
    style_path = Path("./latex_style.mplstyle")
    if not style_path.exists():
        style_path = Path(__file__).resolve().parents[3] / "latex_style.mplstyle"
    if style_path.exists():
        plt.style.use(style_path)


def read_positions(path: Path) -> pd.DataFrame:
    rows = []
    with path.open() as f:
        for line in f:
            parts = line.split()
            if len(parts) != 5:
                continue
            time_s, uav_name, lat, lon, alt = parts
            rows.append(
                {
                    "Time": float(time_s),
                    "UavIndex": int(uav_name.replace("UAV", "")),
                    "Lat": float(lat),
                    "Lon": float(lon),
                    "Alt": float(alt),
                }
            )
    if not rows:
        raise ValueError(f"No UAV position rows found in {path}")
    df = pd.DataFrame(rows)
    lat0 = df["Lat"].mean()
    lon0 = df["Lon"].mean()
    meters_per_deg_lat = 111_320.0
    meters_per_deg_lon = 111_320.0 * math.cos(math.radians(lat0))
    df["EastM"] = (df["Lon"] - lon0) * meters_per_deg_lon
    df["NorthM"] = (df["Lat"] - lat0) * meters_per_deg_lat
    return df


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--output", type=Path, default=Path("docs/figures/uav_xhaul_fading_mobility.pdf"))
    parser.add_argument("--png", type=Path, default=Path("docs/figures/uav_xhaul_fading_mobility.png"))
    args = parser.parse_args()

    xhaul_file = args.run_dir / "xhaul-autonomy-trace.csv"
    pos_file = args.run_dir / "uav-position-trace.tr"
    if not xhaul_file.exists():
        raise FileNotFoundError(xhaul_file)
    if not pos_file.exists():
        raise FileNotFoundError(pos_file)

    load_style()
    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "font.size": 11,
            "axes.labelsize": 12,
            "xtick.labelsize": 9,
            "ytick.labelsize": 9,
            "legend.fontsize": 9,
            "lines.markersize": 4,
        }
    )

    pos = read_positions(pos_file)
    # Downsample dense position traces for a cleaner 3D trajectory rendering.
    pos = pos[pos["Time"].mod(2).eq(0)]
    xhaul = pd.read_csv(xhaul_file)

    colors = ["#4C78A8", "#F28E2B", "#59A14F", "#E15759"]
    fig = plt.figure(figsize=(11.2, 5.9), constrained_layout=True)
    gs = fig.add_gridspec(2, 2, width_ratios=[1.02, 1.48], wspace=0.12)
    ax3d = fig.add_subplot(gs[:, 0], projection="3d")
    ax_switch = fig.add_subplot(gs[0, 1])
    ax_rsrp = fig.add_subplot(gs[1, 1], sharex=ax_switch)

    for uav_idx, group in pos.groupby("UavIndex"):
        color = colors[uav_idx % len(colors)]
        group = group.sort_values("Time")
        ax3d.plot(
            group["EastM"],
            group["NorthM"],
            group["Alt"],
            color=color,
            linewidth=2.2,
            label=f"UAV {uav_idx}",
        )
        ax3d.scatter(
            group["EastM"].iloc[0],
            group["NorthM"].iloc[0],
            group["Alt"].iloc[0],
            color=color,
            marker="o",
            s=34,
        )
        ax3d.scatter(
            group["EastM"].iloc[-1],
            group["NorthM"].iloc[-1],
            group["Alt"].iloc[-1],
            color=color,
            marker="^",
            s=44,
        )

    ax3d.set_title("UAV 3D mobility", pad=8)
    ax3d.set_xlabel("East (m)", labelpad=5)
    ax3d.set_ylabel("North (m)", labelpad=5)
    ax3d.set_zlabel("Altitude (m)", labelpad=5)
    ax3d.tick_params(axis="x", pad=0)
    ax3d.tick_params(axis="y", pad=0)
    ax3d.tick_params(axis="z", pad=2)
    ax3d.view_init(elev=22, azim=-62)
    ax3d.legend(loc="upper left", frameon=True, bbox_to_anchor=(0.02, 0.98))

    for uav_idx, group in xhaul.groupby("UavIndex"):
        color = colors[int(uav_idx) % len(colors)]
        for mode, marker in MODE_MARKERS.items():
            part = group[group["BackhaulMode"].eq(mode)]
            if part.empty:
                continue
            ax_switch.scatter(
                part["Time"],
                part["UavIndex"],
                color=color,
                marker=marker,
                s=34 if mode != "NO_BACKHAUL_AVAILABLE" else 48,
                edgecolors="black" if mode == "NO_BACKHAUL_AVAILABLE" else "none",
                linewidths=0.35,
            )

    ax_switch.set_title("RF-AI Switching xApp Decisions", pad=8)
    ax_switch.set_ylabel("UAV index")
    ax_switch.set_yticks(sorted(xhaul["UavIndex"].unique()))
    ax_switch.grid(True, linestyle="--", alpha=0.42)
    mode_handles = [
        plt.Line2D(
            [0],
            [0],
            marker=marker,
            color="black",
            linestyle="none",
            markerfacecolor="white",
            markersize=7,
            label=mode,
        )
        for mode, marker in MODE_MARKERS.items()
    ]
    uav_handles = [
        plt.Line2D(
            [0],
            [0],
            marker="o",
            color="none",
            linestyle="none",
            markerfacecolor=colors[int(uav_idx) % len(colors)],
            markersize=7,
            label=f"UAV {int(uav_idx)}",
        )
        for uav_idx in sorted(xhaul["UavIndex"].unique())
    ]
    legend1 = ax_switch.legend(handles=mode_handles, loc="upper right", frameon=True, fontsize=7.5)
    ax_switch.add_artist(legend1)
    ax_switch.legend(handles=uav_handles, loc="lower right", frameon=True, fontsize=7.5, ncol=3)

    for uav_idx, group in xhaul.groupby("UavIndex"):
        color = colors[int(uav_idx) % len(colors)]
        group = group.sort_values("Time")
        ax_rsrp.plot(
            group["Time"],
            group["XhaulRsrpDbm"],
            color=color,
            linewidth=1.7,
            label=f"UAV {int(uav_idx)}",
        )

    ax_rsrp.axhline(-100, color="#C44E52", linestyle=":", linewidth=1.6, label="-100 dBm threshold")
    ax_rsrp.set_title("Donor-Link RSRP", pad=8)
    ax_rsrp.set_xlabel("Simulation time (s)")
    ax_rsrp.set_ylabel("Donor RSRP (dBm)")
    ax_rsrp.grid(True, linestyle="--", alpha=0.42)
    ax_rsrp.legend(loc="lower left", frameon=True, fontsize=8)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output)
    fig.savefig(args.png, dpi=300)
    print(f"[saved] {args.output}")
    print(f"[saved] {args.png}")
    print(f"[input] {args.run_dir}")


if __name__ == "__main__":
    main()
