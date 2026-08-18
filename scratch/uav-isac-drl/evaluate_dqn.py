#!/usr/bin/env python3
"""Evaluate a frozen DQN without adding test transitions to replay memory."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

from dqn_agent import DqnAgent
from mock_uav_env import MockUavEnv
from uav_isac_env import UavIsacEnv


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("--environment", choices=["mock", "ns3"], default="ns3")
    parser.add_argument("--seeds", default="9001,9002,9003,9004,9005")
    parser.add_argument("--simulation-time", type=float, default=300.0)
    parser.add_argument("--output", type=Path, default=Path("results/uav-isac-drl/evaluation.csv"))
    args = parser.parse_args()

    agent = DqnAgent.load(args.checkpoint, device="cpu")
    env = MockUavEnv() if args.environment == "mock" else UavIsacEnv(
        simulation_time=args.simulation_time,
        output_root=args.output.parent / "evaluation-episodes",
    )
    rows = []
    try:
        for seed in (int(value) for value in args.seeds.split(",")):
            state, info = env.reset(seed=seed)
            total_reward = 0.0
            steps = 0
            while True:
                action = agent.select_action(state, explore=False)
                state, reward, terminated, truncated, info = env.step(action)
                total_reward += reward
                steps += 1
                if terminated or truncated:
                    break
            rows.append({
                "seed": seed,
                "return": total_reward,
                "steps": steps,
                "pdet": info.get("pdet", ""),
                "rmse_m": info.get("rmse_m", ""),
                "throughput_mbps": info.get("throughput_mbps", ""),
                "delay_ms": info.get("delay_ms", ""),
                "loss_pct": info.get("loss_pct", ""),
                "delta_energy_j": info.get("delta_energy_j", ""),
            })
    finally:
        env.close()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()

