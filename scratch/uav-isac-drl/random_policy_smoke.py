#!/usr/bin/env python3
"""Run synchronized ns-3 episodes with uniformly random actions."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from gymnasium.utils.env_checker import check_env

from uav_isac_env import UavIsacEnv


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--seed", type=int, default=1001)
    parser.add_argument("--simulation-time", type=float, default=40.0)
    parser.add_argument(
        "--output", type=Path, default=Path("results/uav-isac-drl/random-smoke")
    )
    parser.add_argument("--check-env", action="store_true")
    args = parser.parse_args()

    env = UavIsacEnv(simulation_time=args.simulation_time, output_root=args.output)
    if args.check_env:
        check_env(env, skip_render_check=True)
    summaries = []
    try:
        for episode in range(args.episodes):
            observation, info = env.reset(seed=args.seed + episode)
            total_reward = 0.0
            actions = []
            while True:
                action = int(env.action_space.sample())
                observation, reward, terminated, truncated, info = env.step(action)
                total_reward += reward
                actions.append(action)
                if terminated or truncated:
                    break
            summaries.append(
                {
                    "episode": episode,
                    "seed": args.seed + episode,
                    "steps": len(actions),
                    "return": total_reward,
                    "actions": actions,
                    "final": info,
                }
            )
    finally:
        env.close()
    print(json.dumps(summaries, indent=2))


if __name__ == "__main__":
    main()
