from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uav_isac_env import UavIsacEnv


@unittest.skipUnless(os.environ.get("RUN_NS3_INTEGRATION") == "1", "slow ns-3 test")
class Ns3IntegrationTest(unittest.TestCase):
    def test_sensing_speed_and_altitude_actions_change_live_state(self):
        with tempfile.TemporaryDirectory() as output_root:
            env = UavIsacEnv(simulation_time=40.0, output_root=output_root)
            try:
                initial, _ = env.reset(seed=1234)
                sensed, _, _, _, _ = env.step(1)  # role 0 sensing interval decrease
                self.assertLess(sensed[37], initial[37])

                sped, _, _, _, _ = env.step(4)  # role 0 target speed increase
                self.assertGreater(sped[41], sensed[41])

                next_state, _, _, _, _ = env.step(6)  # role 0 altitude increase
                # Role 0 z is observation index 11; target altitude index 45.
                self.assertGreaterEqual(next_state[45], initial[45])
                self.assertGreaterEqual(next_state[11], initial[11])
            finally:
                env.close()

    def test_battery_termination_exits_cleanly_and_writes_summary(self):
        with tempfile.TemporaryDirectory() as output_root:
            env = UavIsacEnv(
                simulation_time=40.0,
                output_root=output_root,
                extra_sim_args={"batteryBudgetJ": 5000.0},
            )
            try:
                _, initial_info = env.reset(seed=4321)
                self.assertFalse(initial_info["terminated"])

                _, _, terminated, truncated, _ = env.step(0)
                self.assertTrue(terminated)
                self.assertFalse(truncated)
                self.assertIsNotNone(env._process)
                self.assertEqual(env._process.poll(), 0)

                summary = env._episode_dir / "run_summary.csv"
                self.assertTrue(summary.is_file())
                log = (env._episode_dir / "ns3.log").read_text(encoding="utf-8")
                self.assertNotIn("NS_FATAL", log)
                self.assertNotIn("disconnected", log.lower())
            finally:
                env.close()


if __name__ == "__main__":
    unittest.main()
