#!/usr/bin/env python3
"""Plot measured UAV/NTN service coverage for clustered underserved-UE runs."""

from __future__ import annotations

from pathlib import Path
import re

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


BASE_DIR = Path("results/nr/tn-ntn")
OUT_DIR = Path("docs/figures")
OUT_DIR.mkdir(parents=True, exist_ok=True)

STYLE = Path("latex_style.mplstyle")
if STYLE.exists():
    plt.style.use(str(STYLE))

RESULT_RE = re.compile(
    r"tn-uav-satellite_ueS1_80_ueS2_(?P<ues2>\d+)_tnGnb_4_ntnGnb_3_"
    r"tnCap_20_ntnCap_15_hyst_2_(?P<controller>rule|rf-ai)-clustered-"
    r"80tn-\d+out-seed1-200s$"
)


def read_ntn_load_series(result_dir: Path, underserved_ues: int) -> pd.DataFrame:
    log = result_dir / "ns3-oran-lm.log"
    if not log.exists():
        return pd.DataFrame(columns=["Time", "CoveragePercent", "NtnLoad"])

    rows = []
    current_time = None
    current_ntn_load = 0
    in_load_block = False

    for line in log.read_text(errors="ignore").splitlines():
        match_time = re.match(r"---- LOAD at t=([\d.]+) ----", line)
        if match_time:
            if in_load_block and current_time is not None:
                rows.append((current_time, min(100.0, 100.0 * current_ntn_load / underserved_ues), current_ntn_load))
            current_time = float(match_time.group(1))
            current_ntn_load = 0
            in_load_block = True
            continue

        if in_load_block:
            match_cell = re.search(r"cell\s+\d+\s+\(NTN\)\s+load=(\d+)", line)
            if match_cell:
                current_ntn_load += int(match_cell.group(1))

    if in_load_block and current_time is not None:
        rows.append((current_time, min(100.0, 100.0 * current_ntn_load / underserved_ues), current_ntn_load))

    return pd.DataFrame(rows, columns=["Time", "CoveragePercent", "NtnLoad"])


def collect_series() -> pd.DataFrame:
    frames = []
    for result_dir in sorted(BASE_DIR.iterdir()):
        if not result_dir.is_dir():
            continue
        match = RESULT_RE.match(result_dir.name)
        if not match:
            continue
        ues2 = int(match.group("ues2"))
        controller = "Rule-based" if match.group("controller") == "rule" else "RF-AI"
        df = read_ntn_load_series(result_dir, ues2)
        if df.empty:
            continue
        df["Controller"] = controller
        df["UnderservedUes"] = ues2
        df["ResultDir"] = result_dir.name
        frames.append(df)
    if not frames:
        raise SystemExit("No clustered coverage series found.")
    return pd.concat(frames, ignore_index=True)


def main() -> None:
    data = collect_series()
    data.to_csv(OUT_DIR / "uav_sample_underserved_coverage_switching_rule_ai.csv", index=False)

    colors = {"Rule-based": "#D9534F", "RF-AI": "#2F73BF"}
    linestyles = {20: "-", 30: "--", 40: ":"}
    markers = {"Rule-based": "s", "RF-AI": "D"}

    fig, ax = plt.subplots(figsize=(11.0, 5.4), constrained_layout=False)
    fig.subplots_adjust(left=0.19, right=0.985, bottom=0.20, top=0.96)

    for controller in ["Rule-based", "RF-AI"]:
        for ues2 in sorted(data["UnderservedUes"].unique()):
            sub = data[(data["Controller"] == controller) & (data["UnderservedUes"] == ues2)]
            if sub.empty:
                continue
            sub = sub.sort_values("Time")
            ax.plot(
                sub["Time"],
                sub["CoveragePercent"],
                color=colors[controller],
                linestyle=linestyles.get(ues2, "-"),
                marker=markers[controller],
                markevery=8,
                label=f"{controller}, {ues2} UEs",
            )

    ax.axhline(90, color="#444444", linestyle=":", linewidth=2.0, label="90\\% target")
    ax.set_xlabel("Simulation time (s)", fontsize=26)
    ax.set_ylabel("Underserved UE coverage (\\%)", fontsize=26, labelpad=10)
    ax.tick_params(axis="both", labelsize=19)
    ax.set_xlim(0, 200)
    ax.set_ylim(0, 105)
    ax.grid(True, linestyle="--", alpha=0.38)
    ax.legend(loc="lower right", ncol=2, frameon=True, fontsize=12)

    for ext in ["png", "pdf"]:
        fig.savefig(
            OUT_DIR / f"uav_sample_underserved_coverage_switching_rule_ai.{ext}",
            dpi=300,
            bbox_inches="tight",
        )
    plt.close(fig)

    summary = (
        data.sort_values("Time")
        .groupby(["Controller", "UnderservedUes"])
        .tail(1)[["Controller", "UnderservedUes", "Time", "NtnLoad", "CoveragePercent"]]
        .sort_values(["UnderservedUes", "Controller"])
    )
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
