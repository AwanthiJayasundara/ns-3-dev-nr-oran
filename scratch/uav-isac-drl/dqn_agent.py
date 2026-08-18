"""Double-DQN implementation for the UAV--ISAC controller."""

from __future__ import annotations

import random
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import torch
from torch import nn


@dataclass(frozen=True)
class DqnConfig:
    state_dim: int = 50
    action_dim: int = 25
    hidden_dim: int = 256
    gamma: float = 0.99
    learning_rate: float = 3e-4
    batch_size: int = 128
    replay_capacity: int = 100_000
    learning_starts: int = 2_000
    train_frequency: int = 1
    target_update_interval: int = 1_000
    gradient_clip_norm: float = 10.0
    epsilon_start: float = 1.0
    epsilon_end: float = 0.05
    epsilon_decay_steps: int = 30_000


class QNetwork(nn.Module):
    def __init__(self, state_dim: int, action_dim: int, hidden_dim: int) -> None:
        super().__init__()
        self.network = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
        )

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        return self.network(state)


class ReplayBuffer:
    def __init__(self, capacity: int, state_dim: int, seed: int = 0) -> None:
        self.capacity = capacity
        self.states = np.empty((capacity, state_dim), dtype=np.float32)
        self.actions = np.empty(capacity, dtype=np.int64)
        self.rewards = np.empty(capacity, dtype=np.float32)
        self.next_states = np.empty((capacity, state_dim), dtype=np.float32)
        self.dones = np.empty(capacity, dtype=np.float32)
        self._size = 0
        self._position = 0
        self._rng = np.random.default_rng(seed)

    def __len__(self) -> int:
        return self._size

    def add(self, state, action, reward, next_state, done) -> None:
        self.states[self._position] = state
        self.actions[self._position] = action
        self.rewards[self._position] = reward
        self.next_states[self._position] = next_state
        self.dones[self._position] = float(done)
        self._position = (self._position + 1) % self.capacity
        self._size = min(self._size + 1, self.capacity)

    def sample(self, batch_size: int, device: torch.device):
        indices = self._rng.integers(0, self._size, size=batch_size)
        return (
            torch.as_tensor(self.states[indices], device=device),
            torch.as_tensor(self.actions[indices], device=device),
            torch.as_tensor(self.rewards[indices], device=device),
            torch.as_tensor(self.next_states[indices], device=device),
            torch.as_tensor(self.dones[indices], device=device),
        )


def double_dqn_targets(
    rewards: torch.Tensor,
    dones: torch.Tensor,
    next_states: torch.Tensor,
    online_network: nn.Module,
    target_network: nn.Module,
    gamma: float,
) -> torch.Tensor:
    """Compute y=r+gamma*(1-done)*Q_target(s',argmax Q_online(s'))."""
    with torch.no_grad():
        next_actions = online_network(next_states).argmax(dim=1, keepdim=True)
        next_q = target_network(next_states).gather(1, next_actions).squeeze(1)
        return rewards + gamma * (1.0 - dones) * next_q


class DqnAgent:
    def __init__(self, config: DqnConfig, seed: int = 0, device: str | None = None) -> None:
        self.config = config
        random.seed(seed)
        np.random.seed(seed)
        torch.manual_seed(seed)
        self.rng = np.random.default_rng(seed)
        self.device = torch.device(device or ("cuda" if torch.cuda.is_available() else "cpu"))
        self.online = QNetwork(config.state_dim, config.action_dim, config.hidden_dim).to(self.device)
        self.target = QNetwork(config.state_dim, config.action_dim, config.hidden_dim).to(self.device)
        self.target.load_state_dict(self.online.state_dict())
        self.target.eval()
        self.optimizer = torch.optim.AdamW(self.online.parameters(), lr=config.learning_rate)
        self.loss_function = nn.SmoothL1Loss()
        self.replay = ReplayBuffer(config.replay_capacity, config.state_dim, seed)
        self.environment_steps = 0
        self.gradient_steps = 0

    @property
    def epsilon(self) -> float:
        fraction = min(1.0, self.environment_steps / self.config.epsilon_decay_steps)
        return self.config.epsilon_start + fraction * (
            self.config.epsilon_end - self.config.epsilon_start
        )

    def select_action(self, state: np.ndarray, explore: bool = True) -> int:
        if explore and self.rng.random() < self.epsilon:
            return int(self.rng.integers(self.config.action_dim))
        state_tensor = torch.as_tensor(state, dtype=torch.float32, device=self.device).unsqueeze(0)
        with torch.no_grad():
            return int(self.online(state_tensor).argmax(dim=1).item())

    def observe(self, state, action, reward, next_state, done) -> None:
        self.replay.add(state, action, reward, next_state, done)
        self.environment_steps += 1

    def optimize(self) -> float | None:
        if self.environment_steps < self.config.learning_starts:
            return None
        if len(self.replay) < self.config.batch_size:
            return None
        if self.environment_steps % self.config.train_frequency:
            return None

        states, actions, rewards, next_states, dones = self.replay.sample(
            self.config.batch_size, self.device
        )
        predicted = self.online(states).gather(1, actions.unsqueeze(1)).squeeze(1)
        targets = double_dqn_targets(
            rewards,
            dones,
            next_states,
            self.online,
            self.target,
            self.config.gamma,
        )
        loss = self.loss_function(predicted, targets)
        self.optimizer.zero_grad(set_to_none=True)
        loss.backward()
        nn.utils.clip_grad_norm_(self.online.parameters(), self.config.gradient_clip_norm)
        self.optimizer.step()
        self.gradient_steps += 1
        if self.environment_steps % self.config.target_update_interval == 0:
            self.target.load_state_dict(self.online.state_dict())
        return float(loss.detach().cpu())

    def save(self, path: Path | str, metadata: dict | None = None) -> None:
        torch.save(
            {
                "config": asdict(self.config),
                "online": self.online.state_dict(),
                "target": self.target.state_dict(),
                "optimizer": self.optimizer.state_dict(),
                "environment_steps": self.environment_steps,
                "gradient_steps": self.gradient_steps,
                "metadata": metadata or {},
            },
            Path(path),
        )

    @classmethod
    def load(cls, path: Path | str, device: str | None = None) -> "DqnAgent":
        checkpoint = torch.load(Path(path), map_location=device or "cpu", weights_only=False)
        agent = cls(DqnConfig(**checkpoint["config"]), device=device)
        agent.online.load_state_dict(checkpoint["online"])
        agent.target.load_state_dict(checkpoint["target"])
        agent.optimizer.load_state_dict(checkpoint["optimizer"])
        agent.environment_steps = checkpoint["environment_steps"]
        agent.gradient_steps = checkpoint["gradient_steps"]
        return agent


__all__ = [
    "DqnAgent",
    "DqnConfig",
    "QNetwork",
    "ReplayBuffer",
    "double_dqn_targets",
]

