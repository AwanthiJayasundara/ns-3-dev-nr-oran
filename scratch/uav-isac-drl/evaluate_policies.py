#!/usr/bin/env python3
"""Equal-seed evaluation for static, random, rule-based, and DQN policies."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.stats import t as student_t

from dqn_agent import DqnAgent
from evaluation_metrics import (
    STATISTICS_METRICS,
    action_changed_state,
    build_episode_row,
)
from offline_rf_benchmark import FEATURES
from uav_isac_env import UavIsacEnv


def load_rf_configuration(path: Path | None) -> dict[str, float]:
    if path is None:
        return {}
    frame = pd.read_csv(path)
    row = frame.iloc[0]
    return {feature: float(row[feature]) for feature in FEATURES if feature in row}


def rule_action(info: dict, step: int) -> int:
    role = step % 4
    if info.get("pdet", 1.0) < 0.97:
        return 1 + role * 6  # decrease sensing interval
    if info.get("rmse_m", 0.0) > 30.0:
        return 1 + role * 6 + 4  # lower target altitude
    return 0


def _paired_rows(
    frame: pd.DataFrame, candidate_method: str, reference_methods: list[str]
) -> list[dict]:
    """Return matched-seed candidate-minus-reference statistics."""
    rows = []
    candidate = frame[frame["method"] == candidate_method].set_index("seed")
    for reference_method in reference_methods:
        reference = frame[frame["method"] == reference_method].set_index("seed")
        shared_seeds = reference.index.intersection(candidate.index)
        for metric in STATISTICS_METRICS:
            differences = (
                candidate.loc[shared_seeds, metric].astype(float)
                - reference.loc[shared_seeds, metric].astype(float)
            )
            count = len(differences)
            std = float(differences.std(ddof=1)) if count > 1 else float("nan")
            critical = float(student_t.ppf(0.975, count - 1)) if count > 1 else float("nan")
            half_width = critical * std / math.sqrt(count) if count > 1 else float("nan")
            mean = float(differences.mean())
            rows.append({
                "method": candidate_method,
                "reference": reference_method,
                "metric": metric,
                "n_pairs": count,
                "mean_difference": mean,
                "ci95_low": mean - half_width,
                "ci95_high": mean + half_width,
            })
    return rows


def write_statistics(frame: pd.DataFrame, output: Path) -> None:
    """Write method CIs and matched-seed policy differences."""
    metrics = STATISTICS_METRICS
    summary_rows = []
    for method, group in frame.groupby("method"):
        for metric in metrics:
            values = group[metric].astype(float)
            count = len(values)
            std = float(values.std(ddof=1)) if count > 1 else float("nan")
            critical = float(student_t.ppf(0.975, count - 1)) if count > 1 else float("nan")
            half_width = critical * std / math.sqrt(count) if count > 1 else float("nan")
            summary_rows.append({
                "method": method,
                "metric": metric,
                "n": count,
                "mean": float(values.mean()),
                "std": std,
                "ci95_low": float(values.mean()) - half_width,
                "ci95_high": float(values.mean()) + half_width,
            })
    pd.DataFrame(summary_rows).to_csv(
        output.with_name(output.stem + "-summary.csv"), index=False
    )

    if "static" not in set(frame["method"]):
        return
    paired_rows = []
    for method in sorted(set(frame["method"]) - {"static"}):
        paired_rows.extend(_paired_rows(frame, method, ["static"]))
    pd.DataFrame(paired_rows).to_csv(
        output.with_name(output.stem + "-paired-vs-static.csv"), index=False
    )

    # Make comparison with the strongest non-DRL policy explicit rather than
    # presenting DQN-versus-static alone.
    if "dqn" in set(frame["method"]):
        references = sorted(set(frame["method"]) - {"dqn"})
        pd.DataFrame(_paired_rows(frame, "dqn", references)).to_csv(
            output.with_name(output.stem + "-paired-dqn-vs-baselines.csv"),
            index=False,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--methods", default="static,random,rule")
    parser.add_argument("--seeds", default="9001,9002,9003,9004,9005")
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--rf-candidate", type=Path)
    parser.add_argument("--simulation-time", type=float, default=300.0)
    parser.add_argument("--output", type=Path, default=Path("results/uav-isac-drl/comparison.csv"))
    args = parser.parse_args()

    methods = args.methods.split(",")
    if "dqn" in methods and args.checkpoint is None:
        parser.error("--checkpoint is required when evaluating dqn")
    dqn = DqnAgent.load(args.checkpoint, device="cpu") if args.checkpoint else None
    seeds = [int(value) for value in args.seeds.split(",")]
    rows = []

    for method in methods:
        extra = load_rf_configuration(args.rf_candidate) if method == "rf-static" else {}
        env = UavIsacEnv(
            simulation_time=args.simulation_time,
            output_root=args.output.parent / "comparison-episodes" / method,
            extra_sim_args=extra,
        )
        try:
            for seed in seeds:
                rng = np.random.default_rng(seed)
                state, info = env.reset(seed=seed)
                total_reward = 0.0
                step = 0
                actions = []
                effective_flags = []
                while True:
                    if method in {"static", "rf-static"}:
                        action = 0
                    elif method == "random":
                        action = int(rng.integers(25))
                    elif method == "rule":
                        action = rule_action(info, step)
                    elif method == "dqn":
                        action = dqn.select_action(state, explore=False)
                    else:
                        raise ValueError(f"Unknown method {method}")
                    previous_state = state
                    state, reward, terminated, truncated, info = env.step(action)
                    actions.append(action)
                    effective_flags.append(
                        action_changed_state(action, previous_state, state)
                    )
                    total_reward += reward
                    step += 1
                    if terminated or truncated:
                        break
                rows.append(
                    build_episode_row(
                        method=method,
                        seed=seed,
                        episode_return=total_reward,
                        actions=actions,
                        effective_flags=effective_flags,
                        final_info=info,
                        episode_dir=env.episode_dir,
                    )
                )
        finally:
            env.close()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    frame = pd.DataFrame(rows)
    write_statistics(frame, args.output)
    print(frame.groupby("method")[STATISTICS_METRICS].agg(["mean", "std"]))
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
