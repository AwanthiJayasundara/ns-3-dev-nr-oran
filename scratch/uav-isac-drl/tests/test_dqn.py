from __future__ import annotations

import sys
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np
import torch
from torch import nn

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dqn_agent import DqnAgent, DqnConfig, ReplayBuffer, double_dqn_targets
from mock_uav_env import MockUavEnv


class FixedNetwork(nn.Module):
    def __init__(self, values):
        super().__init__()
        self.register_buffer("values", torch.tensor(values, dtype=torch.float32))

    def forward(self, states):
        return self.values.repeat(states.shape[0], 1)


class DqnTests(unittest.TestCase):
    def test_double_dqn_uses_online_action_and_target_value(self):
        online = FixedNetwork([1.0, 5.0, 2.0])
        target = FixedNetwork([10.0, 20.0, 30.0])
        targets = double_dqn_targets(
            torch.tensor([2.0, 3.0]),
            torch.tensor([0.0, 1.0]),
            torch.zeros((2, 4)),
            online,
            target,
            gamma=0.5,
        )
        torch.testing.assert_close(targets, torch.tensor([12.0, 3.0]))

    def test_replay_buffer_shapes(self):
        replay = ReplayBuffer(capacity=8, state_dim=4, seed=7)
        for index in range(6):
            state = np.full(4, index, dtype=np.float32)
            replay.add(state, index % 2, float(index), state + 1, index == 5)
        batch = replay.sample(3, torch.device("cpu"))
        self.assertEqual(batch[0].shape, (3, 4))
        self.assertEqual(batch[1].shape, (3,))
        self.assertEqual(len(replay), 6)

    def test_dqn_learns_mock_preferred_action(self):
        config = replace(
            DqnConfig(),
            hidden_dim=64,
            batch_size=32,
            replay_capacity=5_000,
            learning_starts=64,
            target_update_interval=100,
            epsilon_decay_steps=1_000,
        )
        agent = DqnAgent(config, seed=11, device="cpu")
        env = MockUavEnv(episode_length=25)
        for episode in range(70):
            state, _ = env.reset(seed=episode)
            while True:
                action = agent.select_action(state)
                next_state, reward, terminated, truncated, _ = env.step(action)
                done = terminated or truncated
                agent.observe(state, action, reward, next_state, done)
                agent.optimize()
                state = next_state
                if done:
                    break
        state, _ = env.reset(seed=999)
        self.assertEqual(agent.select_action(state, explore=False), 7)


if __name__ == "__main__":
    unittest.main()

