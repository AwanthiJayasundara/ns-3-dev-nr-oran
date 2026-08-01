#!/usr/bin/env python3
"""Compare rule-based and RF-AI UAV TN/NTN switching timelines."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.lines import Line2D


RULE_RUN = Path(
    "results/nr/tn-ntn/ai-dataset-v2/"
    "tn-uav-satellite_ueS1_50_ueS2_50_tnGnb_4_ntnGnb_3_tnCap_20_ntnCap_10_hyst_2_"
    "ai-dataset-v2-logdist-fading-seed1-sat"
)
AI_RUN = Path(
    "results/nr/tn-ntn/"
    "tn-uav-satellite_ueS1_50_ueS2_50_tnGnb_4_ntnGnb_3_tnCap_20_ntnCap_10_hyst_2_ai-switching-sat"
)

MODE_COLORS = {
    "TN_DIRECT": "#59A14F",
    "SATELLITE_FALLBACK": "#F28E2B",
    "NO_BACKHAUL_AVAILABLE": "#E15759",
}


def load_trace(run_dir: Path, controller: str) -> pd.DataFrame:
    trace = run_dir / "xhaul-autonomy-trace.csv"
    if not trace.exists():
        raise FileNotFoundError(trace)
    df = pd.read_csv(trace)
    df["Controller"] = controller
    return df


def first_fallback(df: pd.DataFrame) -> dict[int, float | None]:
    out: dict[int, float | None] = {}
    for uav_idx, group in df.groupby("UavIndex"):
        fallback = group[group["BackhaulMode"].eq("SATELLITE_FALLBACK")]
        out[int(uav_idx)] = None if fallback.empty else float(fallback["Time"].min())
    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rule-run", type=Path, default=RULE_RUN)
    parser.add_argument("--ai-run", type=Path, default=AI_RUN)
    parser.add_argument("--output", type=Path, default=Path("docs/figures/uav_ai_vs_rule_switching_timeline.pdf"))
    parser.add_argument("--png", type=Path, default=Path("docs/figures/uav_ai_vs_rule_switching_timeline.png"))
    parser.add_argument("--csv", type=Path, default=Path("docs/figures/uav_ai_vs_rule_switching_timeline.csv"))
    parser.add_argument(
        "--ai-only",
        action="store_true",
        help="Plot only the RF-AI switching timeline from --ai-run.",
    )
    args = parser.parse_args()

    style_path = Path("./latex_style.mplstyle")
    if not style_path.exists():
        style_path = Path(__file__).resolve().parents[3] / "latex_style.mplstyle"
    if style_path.exists():
        plt.style.use(style_path)

    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "font.size": 15,
            "axes.labelsize": 17,
            "xtick.labelsize": 14,
            "ytick.labelsize": 14,
            "legend.fontsize": 12,
        }
    )

    ai = load_trace(args.ai_run, "RF-AI")
    if args.ai_only:
        data = ai
    else:
        rule = load_trace(args.rule_run, "Rule-based")
        data = pd.concat([rule, ai], ignore_index=True)

    nrows = 1 if args.ai_only else 2
    fig_height = 3.6 if args.ai_only else 5.4
    fig, axes = plt.subplots(nrows, 1, figsize=(8.2, fig_height), sharex=True, constrained_layout=True)
    if args.ai_only:
        axes = [axes]
    for ax, (controller, group) in zip(axes, data.groupby("Controller", sort=False)):
        for mode, color in MODE_COLORS.items():
            part = group[group["BackhaulMode"].eq(mode)]
            if part.empty:
                continue
            ax.scatter(
                part["Time"],
                part["UavIndex"],
                color=color,
                s=34,
                marker="s",
                edgecolors="none",
                label=mode,
            )
        ax.set_title("RF-AI switching xApp" if args.ai_only else controller)
        ax.set_ylabel("UAV index")
        ax.set_yticks(sorted(group["UavIndex"].unique()))
        ax.grid(True, linestyle="--", alpha=0.38)
        ax.set_xlim(0, 120)

    axes[-1].set_xlabel("Simulation time (s)")
    legend_handles = [
        Line2D([0], [0], marker="s", color="none", markerfacecolor=color, markersize=9, label=label)
        for label, color in MODE_COLORS.items()
    ]
    axes[0].legend(handles=legend_handles, loc="upper right", ncol=1, frameon=True)

    ai_first = first_fallback(ai)
    rule_first = {} if args.ai_only else first_fallback(rule)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output)
    fig.savefig(args.png, dpi=300)

    with args.csv.open("w", newline="") as f:
        writer = csv.writer(f)
        if args.ai_only:
            writer.writerow(["UavIndex", "AiFirstFallbackS"])
            for uav_idx in sorted(ai_first):
                a = ai_first.get(uav_idx)
                writer.writerow([uav_idx, "" if a is None else a])
        else:
            writer.writerow(["UavIndex", "RuleFirstFallbackS", "AiFirstFallbackS", "AiLeadTimeS"])
            for uav_idx in sorted(set(rule_first) | set(ai_first)):
                r = rule_first.get(uav_idx)
                a = ai_first.get(uav_idx)
                lead = "" if r is None or a is None else r - a
                writer.writerow([uav_idx, "" if r is None else r, "" if a is None else a, lead])

    print(f"[saved] {args.output}")
    print(f"[saved] {args.png}")
    print(f"[saved] {args.csv}")
    for uav_idx in sorted(set(rule_first) | set(ai_first)):
        if args.ai_only:
            print(f"[summary] UAV {uav_idx}: AI first SAT={ai_first.get(uav_idx)}")
        else:
            print(
                f"[summary] UAV {uav_idx}: rule first SAT={rule_first.get(uav_idx)}, "
                f"AI first SAT={ai_first.get(uav_idx)}"
            )


if __name__ == "__main__":
    main()
