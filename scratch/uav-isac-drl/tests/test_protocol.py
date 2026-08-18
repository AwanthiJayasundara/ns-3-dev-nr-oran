from __future__ import annotations

import json
import socket
import sys
import threading
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from uav_isac_env import UavIsacEnv


def message(step: int, terminated: bool = False):
    return {
        "type": "transition",
        "episode_id": 4,
        "step_id": step,
        "sim_time": 10.0 * (step + 1),
        "reward": 1.25,
        "terminated": terminated,
        "truncated": False,
        "observation": np.zeros(50, dtype=float).tolist(),
    }


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.client, self.server = socket.socketpair()
        self.env = UavIsacEnv()
        self.env._episode_id = 4
        self.env._socket = self.client
        self.env._stream = self.client.makefile("rwb")

    def tearDown(self):
        self.env._process = None
        self.env.close()
        self.server.close()

    def test_step_pairs_ids_and_action(self):
        self.env._current_message = message(0)

        def server():
            stream = self.server.makefile("rwb")
            request = json.loads(stream.readline())
            self.assertEqual(request, {"episode_id": 4, "step_id": 0, "action": 9})
            stream.write((json.dumps(message(1)) + "\n").encode())
            stream.flush()

        thread = threading.Thread(target=server)
        thread.start()
        observation, reward, terminated, truncated, info = self.env.step(9)
        thread.join(timeout=2)
        self.assertEqual(observation.shape, (50,))
        self.assertEqual(reward, 1.25)
        self.assertFalse(terminated)
        self.assertFalse(truncated)
        self.assertEqual(info["step_id"], 1)

    def test_rejects_wrong_observation_size(self):
        invalid = message(0)
        invalid["observation"] = [0.0]
        self.server.sendall((json.dumps(invalid) + "\n").encode())
        with self.assertRaisesRegex(RuntimeError, "Expected 50"):
            self.env._receive_message()


if __name__ == "__main__":
    unittest.main()

