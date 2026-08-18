"""Synchronous Gymnasium wrapper for the ns-3 UAV--ISAC simulation."""

from __future__ import annotations

import json
import socket
import subprocess
import time
from pathlib import Path
from typing import Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces


class UavIsacEnv(gym.Env[np.ndarray, int]):
    """One action controls one 10-second ns-3 simulation window."""

    metadata = {"render_modes": []}
    observation_size = 50
    action_count = 25

    def __init__(
        self,
        repo_root: Path | str | None = None,
        simulation_time: float = 300.0,
        control_interval: float = 10.0,
        output_root: Path | str | None = None,
        connect_timeout: float = 180.0,
        step_timeout: float = 180.0,
        extra_sim_args: dict[str, Any] | None = None,
        no_build: bool = True,
    ) -> None:
        super().__init__()
        self.repo_root = Path(repo_root or Path(__file__).resolve().parents[2]).resolve()
        self.simulation_time = float(simulation_time)
        self.control_interval = float(control_interval)
        self.output_root = Path(output_root or self.repo_root / "results" / "uav-isac-drl")
        self.connect_timeout = float(connect_timeout)
        self.step_timeout = float(step_timeout)
        self.extra_sim_args = dict(extra_sim_args or {})
        self.no_build = no_build

        self.action_space = spaces.Discrete(self.action_count)
        low = np.full(self.observation_size, -1.0, dtype=np.float32)
        high = np.full(self.observation_size, 1.0, dtype=np.float32)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

        self._episode_id = -1
        self._process: subprocess.Popen[str] | None = None
        self._socket: socket.socket | None = None
        self._stream = None
        self._log_handle = None
        self._current_message: dict[str, Any] | None = None

    @staticmethod
    def _free_port() -> int:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.bind(("127.0.0.1", 0))
            return int(sock.getsockname()[1])

    def _simulation_command(self, seed: int, episode_id: int, port: int) -> list[str]:
        arguments: dict[str, Any] = {
            "enableRl": 1,
            "rlPort": port,
            "episodeId": episode_id,
            "RngRun": seed,
            "simulationTime": self.simulation_time,
            "controlInterval": self.control_interval,
        }
        arguments.update(self.extra_sim_args)
        target = "uav-isac-drl " + " ".join(
            f"--{name}={value}" for name, value in arguments.items()
        )
        command = [str(self.repo_root / "ns3"), "run"]
        if self.no_build:
            command.append("--no-build")
        command.extend(["--cwd", str(self._episode_dir), target])
        return command

    def _connect(self, port: int) -> None:
        deadline = time.monotonic() + self.connect_timeout
        last_error: OSError | None = None
        while time.monotonic() < deadline:
            if self._process is not None and self._process.poll() is not None:
                raise RuntimeError(
                    f"ns-3 exited before the RL handshake (code {self._process.returncode}); "
                    f"see {self._episode_dir / 'ns3.log'}"
                )
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(min(1.0, max(0.1, deadline - time.monotonic())))
            try:
                sock.connect(("127.0.0.1", port))
                sock.settimeout(self.step_timeout)
                self._socket = sock
                self._stream = sock.makefile("rwb")
                return
            except OSError as error:
                last_error = error
                sock.close()
                time.sleep(0.05)
        raise TimeoutError(f"Timed out connecting to ns-3 RL bridge: {last_error}")

    def _receive_message(self) -> dict[str, Any]:
        if self._stream is None:
            raise RuntimeError("RL bridge is not connected")
        line = self._stream.readline()
        if not line:
            code = self._process.poll() if self._process is not None else None
            raise RuntimeError(f"ns-3 RL bridge closed unexpectedly (process code {code})")
        message = json.loads(line.decode("utf-8"))
        if message.get("type") != "transition":
            raise RuntimeError(f"Unexpected RL message: {message}")
        observation = np.asarray(message.get("observation"), dtype=np.float32)
        if observation.shape != (self.observation_size,):
            raise RuntimeError(f"Expected 50 observations, got {observation.shape}")
        if not self.observation_space.contains(observation):
            raise RuntimeError("ns-3 returned an observation outside declared bounds")
        return message

    @staticmethod
    def _observation(message: dict[str, Any]) -> np.ndarray:
        return np.asarray(message["observation"], dtype=np.float32)

    @staticmethod
    def _info(message: dict[str, Any]) -> dict[str, Any]:
        return {key: value for key, value in message.items() if key != "observation"}

    def _wait_for_episode_completion(self) -> None:
        """Wait for ns-3 to flush final logs and reject an unclean exit."""
        if self._process is None:
            return
        try:
            return_code = self._process.wait(timeout=self.step_timeout)
        except subprocess.TimeoutExpired as error:
            raise TimeoutError(
                "ns-3 did not exit after its terminal transition; "
                f"see {self._episode_dir / 'ns3.log'}"
            ) from error
        if self._log_handle is not None:
            self._log_handle.flush()
        if return_code != 0:
            raise RuntimeError(
                f"ns-3 exited with code {return_code} after its terminal transition; "
                f"see {self._episode_dir / 'ns3.log'}"
            )

        summary_name = Path(str(self.extra_sim_args.get("summaryFile", "run_summary.csv")))
        summary_path = summary_name if summary_name.is_absolute() else self._episode_dir / summary_name
        if not summary_path.is_file():
            raise RuntimeError(
                f"ns-3 exited cleanly but did not write its final summary: {summary_path}"
            )

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict[str, Any] | None = None,
    ) -> tuple[np.ndarray, dict[str, Any]]:
        super().reset(seed=seed)
        self.close()
        self._episode_id += 1
        simulator_seed = int(seed if seed is not None else self.np_random.integers(1, 2**31 - 1))
        if options and "episode_id" in options:
            self._episode_id = int(options["episode_id"])

        self._episode_dir = self.output_root / f"episode-{self._episode_id:06d}-seed-{simulator_seed}"
        if self._episode_dir.exists() and any(self._episode_dir.iterdir()):
            raise FileExistsError(
                f"Refusing to overwrite an existing episode: {self._episode_dir}. "
                "Choose a new output_root or archive the old run."
            )
        self._episode_dir.mkdir(parents=True, exist_ok=True)
        port = self._free_port()
        command = self._simulation_command(simulator_seed, self._episode_id, port)
        (self._episode_dir / "command.json").write_text(
            json.dumps(command, indent=2) + "\n", encoding="utf-8"
        )
        self._log_handle = (self._episode_dir / "ns3.log").open("w", encoding="utf-8")
        self._process = subprocess.Popen(
            command,
            cwd=self.repo_root,
            stdout=self._log_handle,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self._connect(port)
        message = self._receive_message()
        if int(message["episode_id"]) != self._episode_id or int(message["step_id"]) != 0:
            raise RuntimeError(f"Invalid initial transition identity: {message}")
        self._current_message = message
        return self._observation(message), self._info(message)

    def step(self, action: int) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        if self._current_message is None or self._stream is None:
            raise RuntimeError("Call reset() before step()")
        if not self.action_space.contains(action):
            raise ValueError(f"Action {action} is outside [0, {self.action_count})")
        if self._current_message["terminated"] or self._current_message["truncated"]:
            raise RuntimeError("Episode has ended; call reset()")

        request = {
            "episode_id": int(self._current_message["episode_id"]),
            "step_id": int(self._current_message["step_id"]),
            "action": int(action),
        }
        self._stream.write((json.dumps(request, separators=(",", ":")) + "\n").encode())
        self._stream.flush()
        message = self._receive_message()
        if int(message["episode_id"]) != self._episode_id:
            raise RuntimeError("Received transition for the wrong episode")
        if int(message["step_id"]) != int(self._current_message["step_id"]) + 1:
            raise RuntimeError("Received non-consecutive transition step")
        self._current_message = message
        terminated = bool(message["terminated"])
        truncated = bool(message["truncated"])
        if terminated or truncated:
            self._wait_for_episode_completion()
        return (
            self._observation(message),
            float(message["reward"]),
            terminated,
            truncated,
            self._info(message),
        )

    def close(self) -> None:
        if self._stream is not None:
            try:
                self._stream.close()
            except OSError:
                pass
            self._stream = None
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass
            self._socket = None
        if self._process is not None:
            if self._process.poll() is None:
                self._process.terminate()
                try:
                    self._process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self._process.kill()
                    self._process.wait(timeout=5.0)
            self._process = None
        if self._log_handle is not None:
            self._log_handle.close()
            self._log_handle = None
        self._current_message = None


__all__ = ["UavIsacEnv"]
