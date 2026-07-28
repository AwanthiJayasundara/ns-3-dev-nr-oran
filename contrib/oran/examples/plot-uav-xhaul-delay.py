#!/usr/bin/env python3
"""Plot measured downlink delay over time from UAV xHaul result folders."""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt


SCENARIOS = [
    {
        "label": "TN only",
        "glob": "tn-only_*_clean-seed{seed}",
        "color": "#4C78A8",
        "marker": "D",
        "linestyle": ":",
    },
    {
        "label": "TN + UAV, healthy backhaul",
        "glob": "tn-uav_*_healthy-xhaul-seed{seed}",
        "color": "#59A14F",
        "marker": "^",
        "linestyle": "-",
    },
    {
        "label": "TN + UAV, no satellite",
        "glob": "tn-uav_*_donor-unavailable-no-sat-seed{seed}",
        "color": "#E15759",
        "marker": "s",
        "linestyle": "--",
    },
    {
        "label": "TN + UAV + satellite",
        "glob": "tn-uav-satellite_*_donor-unavailable-sat-seed{seed}",
        "color": "#F28E2B",
        "marker": "o",
        "linestyle": "-.",
    },
]


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else math.nan


def sample_std(values: list[float]) -> float:
    if len(values) < 2:
        return 0.0
    mu = mean(values)
    return math.sqrt(sum((v - mu) ** 2 for v in values) / (len(values) - 1))


def find_result_dir(base_dir: Path, pattern: str) -> Path | None:
    matches = sorted(base_dir.glob(pattern))
    if not matches:
        return None
    if len(matches) > 1:
        print(f"[warn] multiple folders match {pattern}; using {matches[-1]}")
    return matches[-1]


def read_delay_by_time(
    qos_file: Path,
    direction: str,
    include_zero_delay: bool,
    outage_start: float,
    outage_stop: float,
    no_sat_outage_penalty_ms: float,
    apply_no_sat_penalty: bool,
) -> dict[float, float]:
    by_time: dict[float, list[float]] = defaultdict(list)
    with qos_file.open(newline="") as f:
        reader = csv.DictReader(f)
        required = {"Time", "Dir", "Delay"}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{qos_file} missing columns: {sorted(missing)}")

        for row in reader:
            if row["Dir"].strip().upper() != direction:
                continue
            try:
                time_s = float(row["Time"])
                delay_ms = float(row["Delay"]) * 1000.0
                pdr = float(row.get("PDR", "0"))
            except ValueError:
                continue
            if (
                apply_no_sat_penalty
                and no_sat_outage_penalty_ms > 0.0
                and outage_start <= time_s <= outage_stop
                and pdr <= 0.0
            ):
                by_time[time_s].append(no_sat_outage_penalty_ms)
                continue
            if delay_ms == 0.0 and not include_zero_delay:
                continue
            by_time[time_s].append(delay_ms)

    return {time_s: mean(values) for time_s, values in by_time.items()}


def aggregate_scenario(
    base_dir: Path,
    scenario: dict[str, str],
    seeds: list[int],
    direction: str,
    include_zero_delay: bool,
    outage_start: float,
    outage_stop: float,
    no_sat_outage_penalty_ms: float,
):
    seed_series: list[dict[float, float]] = []
    used_dirs: list[Path] = []
    for seed in seeds:
        result_dir = find_result_dir(base_dir, scenario["glob"].format(seed=seed))
        if result_dir is None:
            print(f"[warn] missing {scenario['label']} seed {seed}")
            continue
        qos_file = result_dir / "qos-vs-time.txt"
        if not qos_file.exists():
            print(f"[warn] missing {qos_file}")
            continue
        seed_series.append(
            read_delay_by_time(
                qos_file,
                direction,
                include_zero_delay,
                outage_start,
                outage_stop,
                no_sat_outage_penalty_ms,
                scenario["label"] == "TN + UAV, no satellite",
            )
        )
        used_dirs.append(result_dir)

    all_times = sorted({time_s for series in seed_series for time_s in series})
    rows = []
    for time_s in all_times:
        values = [series[time_s] for series in seed_series if time_s in series]
        rows.append(
            {
                "time_s": time_s,
                "mean_delay_ms": mean(values),
                "std_delay_ms": sample_std(values),
                "num_seeds": len(values),
            }
        )
    return rows, used_dirs


def write_csv(path: Path, plotted: dict[str, list[dict[str, float]]]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Scenario", "TimeS", "MeanDelayMs", "StdDelayMs", "NumSeeds"])
        for label, rows in plotted.items():
            for row in rows:
                writer.writerow(
                    [
                        label,
                        f"{row['time_s']:.3f}",
                        f"{row['mean_delay_ms']:.6f}",
                        f"{row['std_delay_ms']:.6f}",
                        row["num_seeds"],
                    ]
                )


def write_tex(path: Path, pdf_name: str, effective_delay: bool) -> None:
    if effective_delay:
        caption = (
            "Effective downlink delay over time under TN wireless backhaul outage. "
            "No-satellite outage intervals with no delivered downlink packets are represented "
            "using a timeout penalty."
        )
    else:
        caption = (
            "Measured downlink delay over time under TN wireless backhaul outage. "
            "Delay is averaged over delivered downlink packets."
        )
    path.write_text(
        "\\begin{figure}[!t]\n"
        "  \\centering\n"
        f"  \\includegraphics[width=0.92\\linewidth]{{figures/{pdf_name}}}\n"
        f"  \\caption{{{caption}}}\n"
        "  \\label{fig:uav-xhaul-delay-over-time}\n"
        "\\end{figure}\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-dir", default="results/nr/tn-ntn", type=Path)
    parser.add_argument("--output-dir", default="docs/figures", type=Path)
    parser.add_argument("--seeds", default="1,2", help="Comma-separated seed list, e.g., 1,2,3,4")
    parser.add_argument("--direction", default="DL", choices=["DL", "UL"])
    parser.add_argument("--include-zero-delay", action="store_true", help="Include zero-delay rows from intervals with no delivered packets")
    parser.add_argument("--outage-start", default=30.0, type=float)
    parser.add_argument("--outage-stop", default=75.0, type=float)
    parser.add_argument(
        "--no-sat-outage-penalty-ms",
        default=0.0,
        type=float,
        help="If positive, no-satellite outage rows with PDR=0 are plotted as this timeout penalty.",
    )
    parser.add_argument("--prefix", default="uav_xhaul_delay_over_time_seed1_seed2")
    args = parser.parse_args()

    seeds = [int(seed.strip()) for seed in args.seeds.split(",") if seed.strip()]
    args.output_dir.mkdir(parents=True, exist_ok=True)

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
    plotted: dict[str, list[dict[str, float]]] = {}
    max_delay = 0.0

    for scenario in SCENARIOS:
        rows, used_dirs = aggregate_scenario(
            args.base_dir,
            scenario,
            seeds,
            args.direction,
            args.include_zero_delay,
            args.outage_start,
            args.outage_stop,
            args.no_sat_outage_penalty_ms,
        )
        if not rows:
            print(f"[warn] no delay rows for {scenario['label']}")
            continue
        plotted[scenario["label"]] = rows
        times = [row["time_s"] for row in rows]
        means = [row["mean_delay_ms"] for row in rows]
        stds = [row["std_delay_ms"] for row in rows]
        max_delay = max(max_delay, max(means))
        lower = [max(0.0, m - s) for m, s in zip(means, stds)]
        upper = [m + s for m, s in zip(means, stds)]

        ax.plot(
            times,
            means,
            label=scenario["label"],
            color=scenario["color"],
            marker=scenario["marker"],
            markevery=max(1, len(times) // 8),
            linestyle=scenario["linestyle"],
        )
        ax.fill_between(times, lower, upper, color=scenario["color"], alpha=0.10, linewidth=0)
        for result_dir in used_dirs:
            print(f"[input] {scenario['label']}: {result_dir}")

    ax.axvspan(args.outage_start, args.outage_stop, color="#C44E52", alpha=0.10)
    ax.axvline(args.outage_start, color="#C44E52", linestyle=":", linewidth=1.4)
    ax.axvline(args.outage_stop, color="#C44E52", linestyle=":", linewidth=1.4)
    label_y = max(5.0, max_delay * 0.15)
    ax.text(
        (args.outage_start + args.outage_stop) / 2.0,
        label_y,
        "TN wireless backhaul unavailable",
        ha="center",
        va="center",
        color="#8B2E2F",
    )

    ax.set_xlabel("Simulation time (s)")
    ylabel = "Effective delay (ms)" if args.no_sat_outage_penalty_ms > 0.0 else "Delay (ms)"
    ax.set_ylabel(ylabel)
    ax.set_xlim(0, 120)
    ax.set_ylim(bottom=0)
    ax.legend(loc="best", frameon=True, ncol=1)

    png = args.output_dir / f"{args.prefix}.png"
    pdf = args.output_dir / f"{args.prefix}.pdf"
    csv_path = args.output_dir / f"{args.prefix}.csv"
    tex = args.output_dir / f"{args.prefix}.tex"
    fig.savefig(png, dpi=300)
    fig.savefig(pdf)
    write_csv(csv_path, plotted)
    write_tex(tex, pdf.name, args.no_sat_outage_penalty_ms > 0.0)

    print(f"[saved] {png}")
    print(f"[saved] {pdf}")
    print(f"[saved] {csv_path}")
    print(f"[saved] {tex}")


if __name__ == "__main__":
    main()
