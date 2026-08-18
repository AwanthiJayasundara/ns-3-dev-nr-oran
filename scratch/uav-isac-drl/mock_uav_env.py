"""Fast deterministic environment for testing DQN mechanics before ns-3."""

from __future__ import annotations

import gymnasium as gym
import numpy as np
from gymnasium import spaces


class MockUavEnv(gym.Env[np.ndarray, int]):
    def __init__(self, episode_length: int = 40) -> None:
        super().__init__()
        self.episode_length = episode_length
        self.action_space = spaces.Discrete(25)
        self.observation_space = spaces.Box(-1.0, 1.0, shape=(50,), dtype=np.float32)
        self._step = 0
        self._state = np.zeros(50, dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._step = 0
        self._state = self.np_random.uniform(-0.05, 0.05, size=50).astype(np.float32)
        return self._state.copy(), {}

    def step(self, action: int):
        self._step += 1
        target = 7
        reward = 1.0 if action == target else -0.05
        self._state *= 0.95
        self._state[target] = min(1.0, self._state[target] + (0.1 if action == target else -0.01))
        truncated = self._step >= self.episode_length
        return self._state.copy(), reward, False, truncated, {"target_action": target}


__all__ = ["MockUavEnv"]

