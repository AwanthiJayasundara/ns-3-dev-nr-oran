"""Shared, unambiguous episode metrics for UAV--ISAC policy evaluation."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any

import numpy as np


WHOLE_EPISODE_COLUMNS = {
    "Pdet": "episode_pdet",
    "RMSEm": "episode_rmse_m",
    "avgThrMbps": "episode_throughput_mbps",
    "avgDelayMs": "episode_delay_ms",
    "avgLossPct": "episode_loss_pct",
    "EtotJ": "episode_total_energy_j",
}

STATISTICS_METRICS = [
    "episode_return",
    "episode_pdet",
    "episode_rmse_m",
    "episode_throughput_mbps",
    "episode_delay_ms",
    "episode_loss_pct",
    "episode_total_energy_j",
    "mission_duration_s",
    "mission_completed",
    "battery_terminated",
    "effective_control_action_rate",
]


def read_episode_summary(path: Path) -> dict[str, float]:
    """Read the last (normally only) row of an ns-3 whole-run summary."""
    if not path.is_file():
        raise FileNotFoundError(f"Missing whole-episode summary: {path}")
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"Whole-episode summary contains no rows: {path}")
    source = rows[-1]
    missing = [column for column in WHOLE_EPISODE_COLUMNS if column not in source]
    if missing:
        raise ValueError(f"Summary {path} is missing columns: {missing}")
    return {
        output: float(source[column])
        for column, output in WHOLE_EPISODE_COLUMNS.items()
    }


def controlled_observation_index(action: int) -> int | None:
    """Return the observation index changed by a non-zero discrete action."""
    if action == 0:
        return None
    if not 1 <= action < 25:
        raise ValueError(f"Action {action} is outside [0, 25)")
    role = (action - 1) // 6
    operation = (action - 1) % 6
    if operation < 2:
        return 37 + role
    if operation < 4:
        return 41 + role
    return 45 + role


def action_changed_state(
    action: int,
    before: np.ndarray,
    after: np.ndarray,
    *,
    tolerance: float = 1e-7,
) -> bool:
    """Whether a requested control action changed its live control target."""
    index = controlled_observation_index(action)
    if index is None:
        return False
    return not math.isclose(
        float(before[index]), float(after[index]), abs_tol=tolerance, rel_tol=0.0
    )


def action_statistics(actions: list[int], effective_flags: list[bool]) -> dict[str, Any]:
    if len(actions) != len(effective_flags):
        raise ValueError("Every action must have one effectiveness flag")
    noop = sum(action == 0 for action in actions)
    attempted = len(actions) - noop
    effective = sum(
        bool(flag) for action, flag in zip(actions, effective_flags) if action != 0
    )
    clipped = attempted - effective
    rate = effective / attempted if attempted else float("nan")
    return {
        "total_actions": len(actions),
        "noop_actions": noop,
        "attempted_control_actions": attempted,
        "effective_control_actions": effective,
        "clipped_no_effect_actions": clipped,
        "effective_control_action_rate": rate,
    }


def build_episode_row(
    *,
    seed: int,
    episode_return: float,
    actions: list[int],
    effective_flags: list[bool],
    final_info: dict[str, Any],
    episode_dir: Path | None,
    method: str | None = None,
) -> dict[str, Any]:
    """Combine whole-run, final-window, outcome, and action diagnostics."""
    terminated = bool(final_info.get("terminated", False))
    truncated = bool(final_info.get("truncated", False))
    row: dict[str, Any] = {}
    if method is not None:
        row["method"] = method
    row.update(
        {
            "seed": seed,
            "episode_return": float(episode_return),
            "steps": len(actions),
            "mission_duration_s": float(final_info.get("sim_time", float("nan"))),
            "terminated": terminated,
            "truncated": truncated,
            "mission_completed": bool(truncated and not terminated),
            "battery_terminated": terminated,
        }
    )
    if episode_dir is not None:
        row.update(read_episode_summary(episode_dir / "run_summary.csv"))
    else:
        row.update({output: float("nan") for output in WHOLE_EPISODE_COLUMNS.values()})
    row.update(
        {
            "final_window_pdet": final_info.get("pdet", float("nan")),
            "final_window_rmse_m": final_info.get("rmse_m", float("nan")),
            "final_window_throughput_mbps": final_info.get(
                "throughput_mbps", float("nan")
            ),
            "final_window_delay_ms": final_info.get("delay_ms", float("nan")),
            "final_window_loss_pct": final_info.get("loss_pct", float("nan")),
            "final_window_energy_j": final_info.get("delta_energy_j", float("nan")),
        }
    )
    row.update(action_statistics(actions, effective_flags))
    return row


__all__ = [
    "STATISTICS_METRICS",
    "WHOLE_EPISODE_COLUMNS",
    "action_changed_state",
    "action_statistics",
    "build_episode_row",
    "controlled_observation_index",
    "read_episode_summary",
]
