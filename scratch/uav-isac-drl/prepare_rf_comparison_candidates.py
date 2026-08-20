#!/usr/bin/env python3
"""Freeze best-observed and RF-ranked configurations for fair comparison."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pandas as pd

from offline_rf_benchmark import FEATURES


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--design-data",
        type=Path,
        default=Path(
            "results/uav-isac-drl/rf-paper-v1/design_runs_with_corrected_utility.csv"
        ),
    )
    parser.add_argument(
        "--rf-ranked",
        type=Path,
        default=Path("results/uav-isac-drl/rf-paper-v1/top_candidates.csv"),
    )
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("results/uav-isac-drl/rf-paper-v1/comparison-candidates"),
    )
    args = parser.parse_args()
    if args.top_k < 1:
        parser.error("--top-k must be positive")

    design = pd.read_csv(args.design_data)
    ranked = pd.read_csv(args.rf_ranked)
    missing_design = set(FEATURES + ["Jcorrected"]) - set(design.columns)
    missing_ranked = set(FEATURES) - set(ranked.columns)
    if missing_design:
        raise ValueError(f"Design data missing columns: {sorted(missing_design)}")
    if missing_ranked:
        raise ValueError(f"Ranked candidates missing columns: {sorted(missing_ranked)}")
    if len(ranked) < args.top_k:
        raise ValueError(f"Requested top {args.top_k}, but only {len(ranked)} rows exist")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    best_index = design["Jcorrected"].astype(float).idxmax()
    best = design.loc[[best_index], FEATURES]
    best_path = args.output_dir / "best-observed-300.csv"
    best.to_csv(best_path, index=False)

    manifest: dict[str, object] = {
        "selection_rule": "maximum corrected utility among the original 300 design runs",
        "best_observed_source_row": int(best_index),
        "best_observed_rng_run": int(design.loc[best_index, "rngRun"]),
        "best_observed_design_utility": float(design.loc[best_index, "Jcorrected"]),
        "rf_ranked_source": str(args.rf_ranked),
        "files": {"best-observed": str(best_path)},
    }
    for rank in range(1, args.top_k + 1):
        path = args.output_dir / f"rf-rank-{rank}.csv"
        ranked.iloc[[rank - 1]][FEATURES].to_csv(path, index=False)
        manifest["files"][f"rf-rank-{rank}"] = str(path)

    (args.output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
