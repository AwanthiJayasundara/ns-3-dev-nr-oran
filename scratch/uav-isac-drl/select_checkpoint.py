#!/usr/bin/env python3
"""Select a DQN checkpoint using validation seeds only.

Test seeds must not be supplied here. The selected checkpoint path is written
to JSON; the model is not retrained or modified during evaluation.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from dqn_agent import DqnAgent
from mock_uav_env import MockUavEnv
from uav_isac_env import UavIsacEnv


def evaluate(checkpoint: Path, args, candidate_index: int) -> list[dict]:
    agent = DqnAgent.load(checkpoint, device="cpu")
    env = (
        MockUavEnv()
        if args.environment == "mock"
        else UavIsacEnv(
            simulation_time=args.simulation_time,
            control_interval=args.control_interval,
            output_root=args.output.parent
            / "validation-episodes"
            / f"candidate-{candidate_index:03d}",
        )
    )
    rows = []
    try:
        for seed in args.validation_seeds:
            state, info = env.reset(seed=seed)
            episode_return = 0.0
            steps = 0
            while True:
                action = agent.select_action(state, explore=False)
                state, reward, terminated, truncated, info = env.step(action)
                episode_return += reward
                steps += 1
                if terminated or truncated:
                    break
            rows.append(
                {
                    "checkpoint": str(checkpoint.resolve()),
                    "seed": seed,
                    "return": episode_return,
                    "steps": steps,
                    "pdet": info.get("pdet", ""),
                    "rmse_m": info.get("rmse_m", ""),
                    "throughput_mbps": info.get("throughput_mbps", ""),
                    "delay_ms": info.get("delay_ms", ""),
                    "loss_pct": info.get("loss_pct", ""),
                    "delta_energy_j": info.get("delta_energy_j", ""),
                }
            )
    finally:
        env.close()
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoints", nargs="+", type=Path)
    parser.add_argument("--environment", choices=["mock", "ns3"], default="ns3")
    parser.add_argument("--validation-seeds", default="7001,7002,7003,7004,7005")
    parser.add_argument("--simulation-time", type=float, default=300.0)
    parser.add_argument("--control-interval", type=float, default=10.0)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("results/uav-isac-drl/validation/selection.csv"),
    )
    args = parser.parse_args()
    args.validation_seeds = [
        int(value) for value in args.validation_seeds.split(",") if value.strip()
    ]
    if not args.validation_seeds:
        parser.error("at least one validation seed is required")

    rows = []
    for index, checkpoint in enumerate(args.checkpoints):
        rows.extend(evaluate(checkpoint, args, index))

    means = {}
    for checkpoint in args.checkpoints:
        values = [
            row["return"]
            for row in rows
            if row["checkpoint"] == str(checkpoint.resolve())
        ]
        means[str(checkpoint.resolve())] = float(np.mean(values))
    selected = max(means, key=means.get)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    manifest = {
        "selection_metric": "mean validation episodic return",
        "validation_seeds": args.validation_seeds,
        "candidate_mean_returns": means,
        "selected_checkpoint": selected,
        "test_seeds_used": False,
    }
    manifest_path = args.output.with_suffix(".json")
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
