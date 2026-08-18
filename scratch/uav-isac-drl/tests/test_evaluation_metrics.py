from __future__ import annotations

import csv
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from evaluation_metrics import (
    action_changed_state,
    action_statistics,
    build_episode_row,
    controlled_observation_index,
    read_episode_summary,
)


class EvaluationMetricTests(unittest.TestCase):
    def test_action_maps_to_live_control_and_detects_clipping(self):
        state = np.zeros(50, dtype=np.float32)
        changed = state.copy()
        changed[37] = 0.1
        self.assertEqual(controlled_observation_index(1), 37)
        self.assertEqual(controlled_observation_index(4), 41)
        self.assertEqual(controlled_observation_index(6), 45)
        self.assertTrue(action_changed_state(1, state, changed))
        self.assertFalse(action_changed_state(1, state, state.copy()))
        self.assertFalse(action_changed_state(0, state, state.copy()))

    def test_action_statistics_separate_noop_effective_and_clipped(self):
        stats = action_statistics([0, 1, 2, 0], [False, True, False, False])
        self.assertEqual(stats["total_actions"], 4)
        self.assertEqual(stats["noop_actions"], 2)
        self.assertEqual(stats["attempted_control_actions"], 2)
        self.assertEqual(stats["effective_control_actions"], 1)
        self.assertEqual(stats["clipped_no_effect_actions"], 1)
        self.assertEqual(stats["effective_control_action_rate"], 0.5)

    def test_whole_episode_values_are_distinct_from_final_window(self):
        with tempfile.TemporaryDirectory() as directory:
            episode_dir = Path(directory)
            summary = episode_dir / "run_summary.csv"
            headers = ["Pdet", "RMSEm", "avgThrMbps", "avgDelayMs", "avgLossPct", "EtotJ"]
            with summary.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=headers)
                writer.writeheader()
                writer.writerow({
                    "Pdet": 0.8,
                    "RMSEm": 20,
                    "avgThrMbps": 0.002,
                    "avgDelayMs": 9,
                    "avgLossPct": 1,
                    "EtotJ": 12345,
                })
            values = read_episode_summary(summary)
            self.assertEqual(values["episode_total_energy_j"], 12345.0)
            row = build_episode_row(
                method="dqn",
                seed=9,
                episode_return=12.0,
                actions=[1],
                effective_flags=[True],
                final_info={
                    "sim_time": 300,
                    "terminated": False,
                    "truncated": True,
                    "pdet": 1.0,
                    "rmse_m": 50,
                    "throughput_mbps": 0.001,
                    "delay_ms": 12,
                    "loss_pct": 0,
                    "delta_energy_j": 500,
                },
                episode_dir=episode_dir,
            )
            self.assertEqual(row["episode_pdet"], 0.8)
            self.assertEqual(row["final_window_pdet"], 1.0)
            self.assertEqual(row["episode_total_energy_j"], 12345.0)
            self.assertEqual(row["final_window_energy_j"], 500)
            self.assertTrue(row["mission_completed"])
            self.assertFalse(row["battery_terminated"])


if __name__ == "__main__":
    unittest.main()
