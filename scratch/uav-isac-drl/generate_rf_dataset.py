#!/usr/bin/env python3
"""Generate a reproducible random-search dataset for the RF benchmark."""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

from offline_rf_benchmark import FEATURES, sample_candidates


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=int, default=40)
    parser.add_argument("--sampling-seed", type=int, default=20260818)
    parser.add_argument("--first-rng-run", type=int, default=1)
    parser.add_argument("--simulation-time", type=float, default=300.0)
    parser.add_argument("--output", type=Path, default=Path("results/uav-isac-drl/rf-baseline"))
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    output = (repo_root / args.output).resolve() if not args.output.is_absolute() else args.output
    output.mkdir(parents=True, exist_ok=True)
    if not args.dry_run and (output / "run_summary.csv").exists():
        raise FileExistsError(
            f"Refusing to append duplicate runs to {output / 'run_summary.csv'}; "
            "choose a new --output directory or archive the existing campaign."
        )
    candidates = sample_candidates(args.runs, args.sampling_seed)
    candidates.insert(0, "rngRun", range(args.first_rng_run, args.first_rng_run + args.runs))
    candidates.to_csv(output / "configurations.csv", index=False)

    commands = []
    for _, row in candidates.iterrows():
        arguments = {
            "RngRun": int(row["rngRun"]),
            "simulationTime": args.simulation_time,
            "summaryFile": "run_summary.csv",
            "kpiFile": f"kpi-run-{int(row['rngRun']):04d}.csv",
        }
        arguments.update({feature: float(row[feature]) for feature in FEATURES})
        target = "uav-isac-drl " + " ".join(
            f"--{key}={value}" for key, value in arguments.items()
        )
        command = [str(repo_root / "ns3"), "run", "--no-build", "--cwd", str(output), target]
        commands.append(command)
        if not args.dry_run:
            subprocess.run(command, cwd=repo_root, check=True)

    (output / "commands.json").write_text(
        json.dumps(commands, indent=2) + "\n", encoding="utf-8"
    )
    print(f"Wrote configurations and {len(commands)} commands to {output}")


if __name__ == "__main__":
    main()
