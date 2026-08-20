#!/usr/bin/env python3
"""Summarise matched-seed static versus RF-selected ns-3 paper runs."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
from scipy.stats import t as student_t


def scale(series: pd.Series, limits: dict[str, float]) -> pd.Series:
    return ((series - limits["lower"]) /
            (limits["upper"] - limits["lower"])).clip(0.0, 1.0)


def add_corrected_utility(frame: pd.DataFrame, specification: dict) -> pd.DataFrame:
    limits = specification["limits"]
    weights = specification["weights"]
    frame = frame.copy()
    frame["Jcorrected"] = (
        weights["pdet"] * frame["Pdet"].clip(0.0, 1.0)
        + weights["throughput"] * scale(frame["avgThrMbps"], limits["throughput"])
        - weights["rmse"] * scale(frame["RMSEm"], limits["rmse"])
        - weights["delay"] * scale(frame["avgDelayMs"], limits["delay"])
        - weights["energy"] * scale(frame["EtotJ"], limits["energy"])
    )
    energy_columns = [f"UAV{i}_Energy_J" for i in range(4)]
    frame["maxUavEnergyJ"] = frame[energy_columns].max(axis=1)
    frame["batteryFeasible"] = frame["maxUavEnergyJ"] < 100_000.0
    return frame


def ci(values: pd.Series) -> tuple[float, float]:
    n = len(values)
    if n < 2:
        return float("nan"), float("nan")
    half = float(student_t.ppf(0.975, n - 1)) * values.std(ddof=1) / math.sqrt(n)
    return float(values.mean() - half), float(values.mean() + half)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--episodes-root",
        type=Path,
        action="append",
        required=True,
        help="Episode root; repeat to combine independently executed method batches.",
    )
    parser.add_argument("--normalisation", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for episodes_root in args.episodes_root:
        for method_dir in sorted(episodes_root.iterdir()):
            if not method_dir.is_dir():
                continue
            for episode_dir in sorted(method_dir.glob("episode-*-seed-*")):
                summary = episode_dir / "run_summary.csv"
                if not summary.exists():
                    continue
                row = pd.read_csv(summary).iloc[-1].to_dict()
                row["method"] = method_dir.name
                row["seed"] = int(row["rngRun"])
                rows.append(row)
    frame = pd.DataFrame(rows)
    if frame.empty:
        raise RuntimeError("No completed episode summaries were found")
    duplicates = frame.duplicated(subset=["method", "seed"], keep=False)
    if duplicates.any():
        duplicate_keys = frame.loc[duplicates, ["method", "seed"]].drop_duplicates()
        raise RuntimeError(f"Duplicate method/seed episodes found:\n{duplicate_keys}")

    specification = json.loads(args.normalisation.read_text(encoding="utf-8"))
    frame = add_corrected_utility(frame, specification)
    frame.to_csv(args.output_dir / "equal_seed_raw.csv", index=False)

    metrics = [
        "Jcorrected", "Pdet", "RMSEm", "avgThrMbps", "avgDelayMs",
        "avgLossPct", "EtotJ", "maxUavEnergyJ",
    ]
    summaries = []
    for method, group in frame.groupby("method"):
        for metric in metrics:
            low, high = ci(group[metric].astype(float))
            summaries.append({
                "method": method,
                "metric": metric,
                "n": len(group),
                "mean": group[metric].mean(),
                "std": group[metric].std(ddof=1),
                "ci95_low": low,
                "ci95_high": high,
            })
        summaries.append({
            "method": method,
            "metric": "battery_feasibility_rate",
            "n": len(group),
            "mean": group["batteryFeasible"].mean(),
            "std": float("nan"),
            "ci95_low": float("nan"),
            "ci95_high": float("nan"),
        })
    pd.DataFrame(summaries).to_csv(args.output_dir / "equal_seed_summary.csv", index=False)

    pivot = frame.pivot(index="seed", columns="method", values=metrics)
    available_methods = set(pivot.columns.get_level_values(1))
    if "static" not in available_methods:
        raise RuntimeError("A static reference is required")
    candidate_methods = sorted(available_methods - {"static"})
    if not candidate_methods:
        raise RuntimeError("At least one candidate method is required")
    paired = []
    for candidate_method in candidate_methods:
        for metric in metrics:
            paired_values = pivot.loc[:, [(metric, "static"), (metric, candidate_method)]].dropna()
            differences = (
                paired_values[(metric, candidate_method)]
                - paired_values[(metric, "static")]
            )
            low, high = ci(differences)
            paired.append({
                "method": candidate_method,
                "reference": "static",
                "metric": metric,
                "n_pairs": len(differences),
                "mean_difference": differences.mean(),
                "ci95_low": low,
                "ci95_high": high,
            })
    paired_frame = pd.DataFrame(paired)
    paired_frame.to_csv(args.output_dir / "equal_seed_paired.csv", index=False)

    plot_candidate = "rf-static" if "rf-static" in candidate_methods else candidate_methods[0]
    plot_metrics = ["Jcorrected", "Pdet", "RMSEm", "EtotJ"]
    labels = ["Corrected utility", "Detection probability", "RMSE (m)", "Fleet energy (J)"]
    fig, axes = plt.subplots(2, 2, figsize=(8, 6))
    for ax, metric, label in zip(axes.ravel(), plot_metrics, labels):
        values = [
            frame.loc[frame.method == "static", metric],
            frame.loc[frame.method == plot_candidate, metric],
        ]
        ax.boxplot(values, tick_labels=["Default", plot_candidate])
        ax.set_ylabel(label)
        ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    fig.savefig(args.output_dir / "equal_seed_comparison.pdf", bbox_inches="tight")
    fig.savefig(args.output_dir / "equal_seed_comparison.png", dpi=300, bbox_inches="tight")
    plt.close(fig)

    print(pd.DataFrame(summaries).to_string(index=False))
    print("\nPaired candidate-minus-default differences:")
    print(paired_frame.to_string(index=False))


if __name__ == "__main__":
    main()
