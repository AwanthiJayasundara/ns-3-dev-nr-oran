#!/usr/bin/env python3
"""Correct an older comparison CSV using its preserved episode artifacts.

No ns-3 simulations are rerun. Whole-episode KPIs come from run_summary.csv;
outcomes and action effectiveness are reconstructed from the KPI/action logs.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path

import pandas as pd

from evaluate_policies import write_statistics
from evaluation_metrics import action_statistics, read_episode_summary


ACTION_PATTERN = re.compile(
    r"\[RL\] action=(\d+).*?"
    r"sensing=([-+\d.eE]+)->([-+\d.eE]+)\s+"
    r"speed=([-+\d.eE]+)->([-+\d.eE]+)\s+"
    r"altitude=([-+\d.eE]+)->([-+\d.eE]+)"
)
SIMULATION_TIME_PATTERN = re.compile(r"--simulationTime=([-+\d.eE]+)")


def find_episode_dir(root: Path, method: str, seed: int) -> Path:
    matches = sorted((root / method).glob(f"episode-*-seed-{seed}"))
    if len(matches) != 1:
        raise ValueError(
            f"Expected one episode directory for method={method}, seed={seed}; "
            f"found {matches}"
        )
    return matches[0]


def read_duration(episode_dir: Path) -> float:
    path = episode_dir / "control_kpi_log.csv"
    frame = pd.read_csv(path)
    if frame.empty:
        raise ValueError(f"KPI log contains no rows: {path}")
    return float(frame.iloc[-1]["sim_time_s"])


def read_simulation_time(episode_dir: Path) -> float:
    command = json.loads((episode_dir / "command.json").read_text(encoding="utf-8"))
    target = command[-1]
    match = SIMULATION_TIME_PATTERN.search(target)
    if match is None:
        raise ValueError(f"No --simulationTime in {episode_dir / 'command.json'}")
    return float(match.group(1))


def reconstructed_action_statistics(episode_dir: Path, total_actions: int) -> dict:
    text = (episode_dir / "ns3.log").read_text(encoding="utf-8", errors="replace")
    matches = ACTION_PATTERN.findall(text)
    attempted = len(matches)
    if attempted > total_actions:
        raise ValueError(f"More action log rows than steps in {episode_dir}")
    actions = [0] * (total_actions - attempted)
    flags = [False] * len(actions)
    for values in matches:
        action = int(values[0])
        before_after = [float(value) for value in values[1:]]
        changed = any(
            not math.isclose(before_after[index], before_after[index + 1], abs_tol=1e-9)
            for index in (0, 2, 4)
        )
        actions.append(action)
        flags.append(changed)
    return action_statistics(actions, flags)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--episodes-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    episodes_root = args.episodes_root or args.input.parent / "comparison-episodes"
    old_rows = pd.read_csv(args.input).to_dict(orient="records")
    rows = []
    for old in old_rows:
        method = str(old["method"])
        seed = int(old["seed"])
        episode_dir = find_episode_dir(episodes_root, method, seed)
        duration = read_duration(episode_dir)
        simulation_time = read_simulation_time(episode_dir)
        terminated = duration < simulation_time - 1e-9
        truncated = not terminated
        row = {
            "method": method,
            "seed": seed,
            "episode_return": float(old["return"]),
            "steps": int(old["steps"]),
            "mission_duration_s": duration,
            "terminated": terminated,
            "truncated": truncated,
            "mission_completed": truncated and not terminated,
            "battery_terminated": terminated,
            **read_episode_summary(episode_dir / "run_summary.csv"),
            "final_window_pdet": float(old["pdet"]),
            "final_window_rmse_m": float(old["rmse_m"]),
            "final_window_throughput_mbps": float(old["throughput_mbps"]),
            "final_window_delay_ms": float(old["delay_ms"]),
            "final_window_loss_pct": float(old["loss_pct"]),
            "final_window_energy_j": float(old["final_window_energy_j"]),
            **reconstructed_action_statistics(episode_dir, int(old["steps"])),
        }
        rows.append(row)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    frame = pd.DataFrame(rows)
    write_statistics(frame, args.output)
    print(frame.groupby("method").mean(numeric_only=True))
    print(f"Wrote corrected results to {args.output} without rerunning ns-3")


if __name__ == "__main__":
    main()
