#!/usr/bin/env python3
"""Plot measured handover success and ping-pong metrics for clustered UAV runs."""

from __future__ import annotations

from collections import defaultdict
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


def read_handover_success(path: Path) -> list[tuple[float, int, int]]:
    events: list[tuple[float, int, int]] = []
    trace = path / "handover-trace.tr"
    if trace.exists():
        for line in trace.read_text(errors="ignore").splitlines():
            match = re.search(r"^([\d.]+)\s+IMSI=(\d+)\s+TargetCell=(\d+)", line)
            if match:
                events.append((float(match.group(1)), int(match.group(2)), int(match.group(3))))
    if events:
        return events

    log = path / "ns3-oran-lm.log"
    if not log.exists():
        return events
    for line in log.read_text(errors="ignore").splitlines():
        match = re.search(r"TRACE HO_SUCCESS Time=([\d.]+) IMSI=(\d+) TargetCell=(\d+)", line)
        if match:
            events.append((float(match.group(1)), int(match.group(2)), int(match.group(3))))
    return events


def count_rrc_failures(path: Path) -> int:
    failure_trace = path / "handover-failure-trace.tr"
    if not failure_trace.exists():
        return 0
    return sum(1 for line in failure_trace.read_text(errors="ignore").splitlines() if line.strip())


def count_ping_pong(events: list[tuple[float, int, int]], window_s: float = 10.0) -> int:
    by_ue: dict[int, list[tuple[float, int]]] = defaultdict(list)
    for t, imsi, cell in events:
        by_ue[imsi].append((t, cell))

    count = 0
    for ue_events in by_ue.values():
        ue_events.sort()
        for i in range(len(ue_events) - 2):
            t0, c0 = ue_events[i]
            _, c1 = ue_events[i + 1]
            t2, c2 = ue_events[i + 2]
            if c0 == c2 and c0 != c1 and (t2 - t0) <= window_s:
                count += 1
    return count


def collect_metrics() -> pd.DataFrame:
    rows = []
    for result_dir in sorted(BASE_DIR.iterdir()):
        if not result_dir.is_dir():
            continue
        match = RESULT_RE.match(result_dir.name)
        if not match:
            continue
        ues2 = int(match.group("ues2"))
        controller = "Rule-based" if match.group("controller") == "rule" else "RF-AI"
        events = read_handover_success(result_dir)
        failures = count_rrc_failures(result_dir)
        ping_pong = count_ping_pong(events)
        attempts = len(events) + failures
        success_rate = 100.0 * len(events) / attempts if attempts else 0.0
        ping_pong_pct = 100.0 * ping_pong / len(events) if events else 0.0
        rows.append(
            {
                "Controller": controller,
                "UnderservedUes": ues2,
                "SuccessfulHoCommands": len(events),
                "RrcHoFailures": failures,
                "HandoverSuccessRatePercent": success_rate,
                "PingPongEvents": ping_pong,
                "PingPongPercentOfSuccessfulHo": ping_pong_pct,
                "ResultDir": result_dir.name,
            }
        )
    if not rows:
        raise SystemExit("No clustered rule/RF-AI result folders found.")
    return pd.DataFrame(rows).sort_values(["UnderservedUes", "Controller"])


def main() -> None:
    df = collect_metrics()
    df.to_csv(OUT_DIR / "uav_sample_ho_success_pingpong_rule_ai.csv", index=False)

    demand = np.array(sorted(df["UnderservedUes"].unique()))
    x = np.arange(len(demand))
    width = 0.34
    colors = {"Rule-based": "#D9534F", "RF-AI": "#2F73BF"}

    fig, axes = plt.subplots(1, 2, figsize=(13.8, 5.4), constrained_layout=False)
    fig.subplots_adjust(left=0.12, right=0.985, bottom=0.25, top=0.86, wspace=0.22)

    for offset, controller in [(-width / 2, "Rule-based"), (width / 2, "RF-AI")]:
        sub = df[df["Controller"] == controller].set_index("UnderservedUes").reindex(demand)
        axes[0].bar(
            x + offset,
            sub["HandoverSuccessRatePercent"],
            width,
            label=controller,
            color=colors[controller],
            edgecolor="black",
            linewidth=0.5,
        )
        axes[1].bar(
            x + offset,
            sub["PingPongPercentOfSuccessfulHo"],
            width,
            label=controller,
            color=colors[controller],
            edgecolor="black",
            linewidth=0.5,
        )

    axes[0].set_ylabel("Handover success rate (\\%)")
    axes[0].set_ylim(95, 101)
    axes[1].set_ylabel("Ping-pong events (\\%)")
    ymax = max(2.0, df["PingPongPercentOfSuccessfulHo"].max() * 1.35)
    axes[1].set_ylim(0, ymax)

    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels([str(v) for v in demand])
        ax.set_xlabel("Underserved UEs")
        ax.grid(True, axis="y", linestyle="--", alpha=0.45)

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, frameon=True, bbox_to_anchor=(0.55, 0.98))
    fig.text(0.30, 0.055, "(a)", ha="center", va="center", fontsize=22)
    fig.text(0.755, 0.055, "(b)", ha="center", va="center", fontsize=22)

    for ext in ["png", "pdf"]:
        fig.savefig(OUT_DIR / f"uav_sample_ho_success_pingpong_rule_ai.{ext}", dpi=300)
    plt.close(fig)
    print(df.to_string(index=False))


if __name__ == "__main__":
    main()
