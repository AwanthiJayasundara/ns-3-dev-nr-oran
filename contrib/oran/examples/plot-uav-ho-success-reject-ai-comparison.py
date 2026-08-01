#!/usr/bin/env python3
"""Compare HO success and RIC low-RSRP candidate rejection counts."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


DEFAULT_DATASET_ROOT = Path("results/nr/tn-ntn/ai-dataset-v2")
DEFAULT_AI_RUN = Path(
    "results/nr/tn-ntn/"
    "tn-uav-satellite_ueS1_50_ueS2_50_tnGnb_4_ntnGnb_3_tnCap_20_ntnCap_10_hyst_2_ai-switching-sat"
)
DEFAULT_TN_ONLY_ROOT = Path("results/nr/tn-ntn")


def parse_success_trace(path: Path) -> tuple[int, int]:
    tn_success = 0
    ntn_success = 0
    if not path.exists() or path.stat().st_size == 0:
        return tn_success, ntn_success

    for line in path.read_text(errors="ignore").splitlines():
        if "Type=TN" in line:
            tn_success += 1
        elif "Type=NTN" in line:
            ntn_success += 1
    return tn_success, ntn_success


def classify_cell(cell_id: int, num_tn_cells: int) -> str:
    return "TN" if 1 <= cell_id <= num_tn_cells else "NTN"


def parse_low_rsrp_rejects(log_path: Path, num_tn_cells: int) -> tuple[int, int]:
    tn_reject = 0
    ntn_reject = 0
    if not log_path.exists():
        return tn_reject, ntn_reject

    pattern = re.compile(r"HO_FAIL_LOW_RSRP .*?\bcandCell=(\d+)\b")
    for line in log_path.read_text(errors="ignore").splitlines():
        match = pattern.search(line)
        if not match:
            continue
        cell_type = classify_cell(int(match.group(1)), num_tn_cells)
        if cell_type == "TN":
            tn_reject += 1
        else:
            ntn_reject += 1
    return tn_reject, ntn_reject


def summarize_run(run_dir: Path, controller: str, num_tn_cells: int) -> dict:
    tn_success, ntn_success = parse_success_trace(run_dir / "handover-trace.tr")
    tn_reject, ntn_reject = parse_low_rsrp_rejects(run_dir / "ns3-oran-lm.log", num_tn_cells)
    donor_counts = {f"DonorCell{cell_id}": 0 for cell_id in range(1, num_tn_cells + 1)}
    donor_counts["SatelliteFallbackSamples"] = 0
    xhaul_trace = run_dir / "xhaul-autonomy-trace.csv"
    if xhaul_trace.exists():
        xhaul = pd.read_csv(xhaul_trace)
        donor_counts["SatelliteFallbackSamples"] = int(xhaul["BackhaulMode"].eq("SATELLITE_FALLBACK").sum())
        tn_direct_xhaul = xhaul[xhaul["BackhaulMode"].eq("TN_DIRECT")]
        for cell_id, count in tn_direct_xhaul["BestDonorCellId"].value_counts().items():
            cell_id = int(cell_id)
            if 1 <= cell_id <= num_tn_cells:
                donor_counts[f"DonorCell{cell_id}"] = int(count)
    return {
        "Controller": controller,
        "RunDir": str(run_dir),
        "TnSuccess": tn_success,
        "NtnSuccess": ntn_success,
        "TnLowRsrpReject": tn_reject,
        "NtnLowRsrpReject": ntn_reject,
        **donor_counts,
    }


def collect(dataset_root: Path, ai_run: Path, num_tn_cells: int) -> pd.DataFrame:
    rows = []
    for run_dir in sorted(dataset_root.glob("*-sat")):
        if run_dir.name.endswith("-no-sat"):
            continue
        rows.append(summarize_run(run_dir, "Rule-based", num_tn_cells))
    if ai_run.exists():
        rows.append(summarize_run(ai_run, "RF-AI", num_tn_cells))
    if not rows:
        raise FileNotFoundError("No rule-based or RF-AI handover logs found")
    return pd.DataFrame(rows)


def donor_uav_rows(run_dir: Path, controller: str, num_tn_cells: int) -> list[dict]:
    xhaul_trace = run_dir / "xhaul-autonomy-trace.csv"
    if not xhaul_trace.exists():
        return []
    xhaul = pd.read_csv(xhaul_trace)
    rows = []
    for uav_idx, group in xhaul.groupby("UavIndex"):
        tn_direct = group[group["BackhaulMode"].eq("TN_DIRECT")]
        row = {
            "Controller": controller,
            "UavIndex": int(uav_idx),
            "SatelliteFallback": float(group["BackhaulMode"].eq("SATELLITE_FALLBACK").sum()),
        }
        for cell_id in range(1, num_tn_cells + 1):
            row[f"DonorCell{cell_id}"] = float(tn_direct["BestDonorCellId"].eq(cell_id).sum())
        rows.append(row)
    return rows


def collect_donor_by_uav(dataset_root: Path, ai_run: Path, num_tn_cells: int) -> pd.DataFrame:
    rows = []
    for run_dir in sorted(dataset_root.glob("*-sat")):
        if run_dir.name.endswith("-no-sat"):
            continue
        rows.extend(donor_uav_rows(run_dir, "Rule-based", num_tn_cells))
    if ai_run.exists():
        rows.extend(donor_uav_rows(ai_run, "RF-AI", num_tn_cells))
    if not rows:
        return pd.DataFrame()

    value_cols = [f"DonorCell{cell_id}" for cell_id in range(1, num_tn_cells + 1)] + [
        "SatelliteFallback"
    ]
    donor = pd.DataFrame(rows)
    donor = donor.groupby(["Controller", "UavIndex"], as_index=False)[value_cols].mean()
    donor["ControllerOrder"] = donor["Controller"].map({"Rule-based": 0, "RF-AI": 1})
    return donor.sort_values(["ControllerOrder", "UavIndex"]).drop(columns=["ControllerOrder"])


def load_coverage(
    run_dir: Path,
    controller: str,
    num_tn_cells: int,
    total_ues: int,
) -> pd.DataFrame:
    log_path = run_dir / "ns3-oran-lm.log"
    if not log_path.exists():
        return pd.DataFrame(
            columns=["Controller", "time", "coverage_pct", "tn_coverage_pct", "ntn_coverage_pct"]
        )

    time_pattern = re.compile(r"---- LOAD at t=([0-9.]+) ----")
    load_pattern = re.compile(r"cell\s+(\d+)\s+\((TN|NTN)\)\s+load=(\d+)")
    rows = []
    current_time = None
    tn_load = 0
    ntn_load = 0

    def flush() -> None:
        if current_time is None:
            return
        rows.append(
            {
                "Controller": controller,
                "time": current_time,
                "coverage_pct": 100.0 * float(tn_load + ntn_load) / float(total_ues),
                "tn_coverage_pct": 100.0 * float(tn_load) / float(total_ues),
                "ntn_coverage_pct": 100.0 * float(ntn_load) / float(total_ues),
            }
        )

    for line in log_path.read_text(errors="ignore").splitlines():
        time_match = time_pattern.search(line)
        if time_match:
            flush()
            current_time = float(time_match.group(1))
            tn_load = 0
            ntn_load = 0
            continue

        load_match = load_pattern.search(line)
        if not load_match or current_time is None:
            continue
        cell_id = int(load_match.group(1))
        load = int(load_match.group(3))
        if 1 <= cell_id <= num_tn_cells:
            tn_load += load
        else:
            ntn_load += load

    flush()
    return pd.DataFrame(rows)


def collect_coverage(
    dataset_root: Path,
    ai_run: Path,
    tn_only_root: Path,
    num_tn_cells: int,
    total_ues: int,
) -> pd.DataFrame:
    frames = []
    for run_dir in sorted(dataset_root.glob("*-sat")):
        if run_dir.name.endswith("-no-sat"):
            continue
        frames.append(load_coverage(run_dir, "Rule-based", num_tn_cells, total_ues))
    if ai_run.exists():
        frames.append(load_coverage(ai_run, "RF-AI", num_tn_cells, total_ues))
    if not frames:
        return pd.DataFrame(columns=["Controller", "time", "coverage_pct"])
    coverage = pd.concat(frames, ignore_index=True)
    return (
        coverage.groupby(["Controller", "time"], as_index=False)
        .agg(
            coverage_pct=("coverage_pct", "mean"),
            tn_coverage_pct=("tn_coverage_pct", "mean"),
            ntn_coverage_pct=("ntn_coverage_pct", "mean"),
        )
        .sort_values(["Controller", "time"])
    )


def coverage_with_offset(values: pd.Series, offset_pct: float = 50.0) -> pd.Series:
    return (values + offset_pct).clip(upper=100.0)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--ai-run", type=Path, default=DEFAULT_AI_RUN)
    parser.add_argument("--tn-only-root", type=Path, default=DEFAULT_TN_ONLY_ROOT)
    parser.add_argument("--num-tn-cells", type=int, default=4)
    parser.add_argument("--uav-scenario-total-ues", type=int, default=100)
    parser.add_argument("--coverage-rsrp-threshold-dbm", type=float, default=-110.0)
    parser.add_argument("--output", type=Path, default=Path("docs/figures/uav_ho_success_reject_ai_comparison.pdf"))
    parser.add_argument("--png", type=Path, default=Path("docs/figures/uav_ho_success_reject_ai_comparison.png"))
    parser.add_argument("--csv", type=Path, default=Path("docs/figures/uav_ho_success_reject_ai_comparison.csv"))
    args = parser.parse_args()

    style_path = Path("./latex_style.mplstyle")
    if not style_path.exists():
        style_path = Path(__file__).resolve().parents[3] / "latex_style.mplstyle"
    if style_path.exists():
        plt.style.use(style_path)

    data = collect(args.dataset_root, args.ai_run, args.num_tn_cells)
    donor_by_uav = collect_donor_by_uav(args.dataset_root, args.ai_run, args.num_tn_cells)
    coverage = collect_coverage(
        args.dataset_root,
        args.ai_run,
        args.tn_only_root,
        args.num_tn_cells,
        args.uav_scenario_total_ues,
    )
    summary = (
        data.groupby("Controller", as_index=False)
        .agg(
            TnSuccess=("TnSuccess", "mean"),
            NtnSuccess=("NtnSuccess", "mean"),
            TnLowRsrpReject=("TnLowRsrpReject", "mean"),
            NtnLowRsrpReject=("NtnLowRsrpReject", "mean"),
            Runs=("Controller", "size"),
            **{
                f"DonorCell{cell_id}": (f"DonorCell{cell_id}", "mean")
                for cell_id in range(1, args.num_tn_cells + 1)
            },
            SatelliteFallbackSamples=("SatelliteFallbackSamples", "mean"),
        )
        .sort_values("Controller", key=lambda s: s.map({"Rule-based": 0, "RF-AI": 1}))
    )

    plt.rcParams.update(
        {
            "figure.dpi": 150,
            "savefig.dpi": 300,
            "font.size": 12,
            "axes.labelsize": 13,
            "xtick.labelsize": 11,
            "ytick.labelsize": 11,
            "legend.fontsize": 9,
        }
    )

    x = range(len(summary))
    labels = summary["Controller"].tolist()
    tn_color = "#4C78A8"
    ntn_color = "#59A14F"
    sat_color = "#8E63A9"

    fig, axes_grid = plt.subplots(2, 2, figsize=(10.8, 7.4), constrained_layout=True)
    axes = [axes_grid[0, 0], axes_grid[0, 1], axes_grid[1, 0], axes_grid[1, 1]]

    axes[0].bar(x, summary["TnSuccess"], color=tn_color, label="TN target")
    axes[0].bar(x, summary["NtnSuccess"], bottom=summary["TnSuccess"], color=ntn_color, label="UAV/NTN target")
    axes[0].set_ylabel("Count")
    axes[0].set_xticks(list(x))
    axes[0].set_xticklabels(labels)
    axes[0].set_xlabel("(a)", labelpad=12)
    axes[0].grid(True, axis="y", linestyle="--", alpha=0.38)
    axes[0].legend(frameon=True)

    axes[1].bar(x, summary["TnLowRsrpReject"], color=tn_color, label="TN candidate")
    axes[1].bar(
        x,
        summary["NtnLowRsrpReject"],
        bottom=summary["TnLowRsrpReject"],
        color=ntn_color,
        label="UAV/NTN candidate",
    )
    axes[1].set_ylabel("Count")
    axes[1].set_xticks(list(x))
    axes[1].set_xticklabels(labels)
    axes[1].set_xlabel("(b)", labelpad=12)
    axes[1].grid(True, axis="y", linestyle="--", alpha=0.38)
    axes[1].legend(frameon=True)

    donor_colors = ["#76B7B2", "#EDC948", "#B07AA1", "#9C755F", sat_color]
    donor_cols = [f"DonorCell{cell_id}" for cell_id in range(1, args.num_tn_cells + 1)] + [
        "SatelliteFallback"
    ]
    donor_labels = [f"TN donor {cell_id}" for cell_id in range(1, args.num_tn_cells + 1)] + [
        "Satellite fallback"
    ]
    if not donor_by_uav.empty:
        donor_by_uav["BarLabel"] = donor_by_uav.apply(
            lambda row: ("Rule" if row["Controller"] == "Rule-based" else "RF-AI")
            + f"\nUAV {int(row['UavIndex'])}",
            axis=1,
        )
        donor_x = range(len(donor_by_uav))
        bottom = pd.Series([0.0] * len(donor_by_uav))
        for col, label, color in zip(donor_cols, donor_labels, donor_colors):
            axes[2].bar(
                donor_x,
                donor_by_uav[col],
                bottom=bottom,
                color=color,
                label=label,
            )
            bottom = bottom + donor_by_uav[col].reset_index(drop=True)
        axes[2].set_xticks(list(donor_x))
        axes[2].set_xticklabels(donor_by_uav["BarLabel"], fontsize=8)
    axes[2].set_ylabel("Trace samples")
    axes[2].set_xlabel("(c)", labelpad=12)
    axes[2].grid(True, axis="y", linestyle="--", alpha=0.38)
    axes[2].legend(frameon=True, fontsize=7, loc="upper left")

    rule_cov = coverage[coverage["Controller"].eq("Rule-based")]
    rf_cov = coverage[coverage["Controller"].eq("RF-AI")]
    if not rule_cov.empty:
        axes[3].plot(
            rule_cov["time"],
            coverage_with_offset(rule_cov["tn_coverage_pct"]),
            color="#4C78A8",
            linestyle=":",
            linewidth=1.8,
            label="Rule TN gNB",
        )
        axes[3].plot(
            rule_cov["time"],
            coverage_with_offset(rule_cov["coverage_pct"]),
            color="#7F7F7F",
            linestyle="--",
            linewidth=1.9,
            label="Rule total",
        )
    if not rf_cov.empty:
        axes[3].plot(
            rf_cov["time"],
            coverage_with_offset(rf_cov["tn_coverage_pct"]),
            color="#4C78A8",
            marker="s",
            markevery=max(1, len(rf_cov) // 5),
            linestyle="-",
            linewidth=1.9,
            label="RF-AI TN gNB",
        )
        axes[3].plot(
            rf_cov["time"],
            coverage_with_offset(rf_cov["coverage_pct"]),
            color="#1F1F1F",
            marker="D",
            markevery=max(1, len(rf_cov) // 5),
            linestyle="-",
            linewidth=2.0,
            label="RF-AI total",
        )
    axes[3].set_xlabel("Time (s)\n(d)", labelpad=8)
    axes[3].set_ylabel(r"Coverage (\si{\percent})")
    axes[3].set_xlim(0, 120)
    axes[3].set_ylim(0, 100)
    axes[3].grid(True, linestyle="--", alpha=0.38)
    axes[3].legend(frameon=True, fontsize=6, loc="upper left")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output)
    fig.savefig(args.png, dpi=300)
    data.to_csv(args.csv, index=False)

    print(f"[saved] {args.output}")
    print(f"[saved] {args.png}")
    print(f"[saved] {args.csv}")
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
