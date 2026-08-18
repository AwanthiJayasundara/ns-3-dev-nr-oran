#!/usr/bin/env python3
"""Train Double DQN in the mock environment or synchronized ns-3."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from dqn_agent import DqnAgent, DqnConfig
from mock_uav_env import MockUavEnv
from uav_isac_env import UavIsacEnv


def make_env(args):
    if args.environment == "mock":
        return MockUavEnv(episode_length=args.mock_episode_length)
    return UavIsacEnv(
        simulation_time=args.simulation_time,
        control_interval=args.control_interval,
        output_root=args.output / "episodes",
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--environment", choices=["mock", "ns3"], default="mock")
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--simulation-time", type=float, default=300.0)
    parser.add_argument("--control-interval", type=float, default=10.0)
    parser.add_argument("--mock-episode-length", type=int, default=40)
    parser.add_argument("--output", type=Path, default=Path("results/uav-isac-drl/training"))
    parser.add_argument("--learning-starts", type=int)
    parser.add_argument("--batch-size", type=int)
    parser.add_argument("--gamma", type=float)
    parser.add_argument("--learning-rate", type=float)
    parser.add_argument("--hidden-dim", type=int)
    parser.add_argument("--target-update-interval", type=int)
    parser.add_argument("--epsilon-decay-steps", type=int)
    parser.add_argument(
        "--checkpoint-every",
        type=int,
        default=10,
        help="Preserve a candidate checkpoint every N episodes (0 disables it)",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)

    config = DqnConfig()
    if args.learning_starts is not None:
        config = replace(config, learning_starts=args.learning_starts)
    if args.batch_size is not None:
        config = replace(config, batch_size=args.batch_size)
    for argument, field in (
        (args.gamma, "gamma"),
        (args.learning_rate, "learning_rate"),
        (args.hidden_dim, "hidden_dim"),
        (args.target_update_interval, "target_update_interval"),
        (args.epsilon_decay_steps, "epsilon_decay_steps"),
    ):
        if argument is not None:
            config = replace(config, **{field: argument})
    agent = DqnAgent(config, seed=args.seed)
    env = make_env(args)
    rows = []
    best_return = -float("inf")
    try:
        for episode in range(args.episodes):
            state, _ = env.reset(seed=args.seed * 100_000 + episode)
            episode_return = 0.0
            losses = []
            action_counts = np.zeros(config.action_dim, dtype=int)
            while True:
                action = agent.select_action(state, explore=True)
                next_state, reward, terminated, truncated, _ = env.step(action)
                done = terminated or truncated
                agent.observe(state, action, reward, next_state, done)
                loss = agent.optimize()
                if loss is not None:
                    losses.append(loss)
                action_counts[action] += 1
                episode_return += reward
                state = next_state
                if done:
                    break

            row = {
                "episode": episode,
                "return": episode_return,
                "epsilon": agent.epsilon,
                "mean_loss": float(np.mean(losses)) if losses else "",
                "environment_steps": agent.environment_steps,
                "greedy_action": agent.select_action(state, explore=False),
                "action_counts": json.dumps(action_counts.tolist()),
            }
            rows.append(row)
            print(json.dumps(row))
            if episode_return > best_return:
                best_return = episode_return
                # This is diagnostic only: training return must not select the paper model.
                agent.save(
                    args.output / "best-training-return.pt",
                    {"episode": episode, "training_return": best_return},
                )
            agent.save(args.output / "latest.pt", {"episode": episode, "return": episode_return})
            if args.checkpoint_every and (
                (episode + 1) % args.checkpoint_every == 0 or episode + 1 == args.episodes
            ):
                agent.save(
                    args.output / f"checkpoint-episode-{episode + 1:06d}.pt",
                    {"episode": episode, "training_return": episode_return},
                )
    finally:
        env.close()

    with (args.output / "training.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    (args.output / "config.json").write_text(
        json.dumps(vars(args), indent=2, default=str) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
