from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
try:
    from stable_baselines3 import PPO  # type: ignore
except Exception:  # pragma: no cover - fallback for CI without heavy deps
    class PPO:  # simple stub used when stable-baselines3 is unavailable
        def __init__(self, policy, env, verbose=0):
            self.env = env

        def set_env(self, env):
            self.env = env

        def learn(self, total_timesteps: int):  # noqa: D401 - dummy
            """No-op learning step."""

        def predict(self, obs, deterministic: bool = True):
            shape = getattr(self.env.action_space, "shape", None)
            action = np.zeros(shape, dtype=np.float32)
            return action, None

        def save(self, path: str) -> None:
            pass

        @classmethod
        def load(cls, path: str):
            return cls("MlpPolicy", None)
import gymnasium as gym


class _StaticEnv(gym.Env):
    """Minimal environment just to initialize PPO."""

    def __init__(self, observation_space: gym.Space, action_space: gym.Space) -> None:
        self.observation_space = observation_space
        self.action_space = action_space

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        obs = np.zeros(self.observation_space.shape, dtype=np.float32)
        return obs, {}

    def step(self, action):
        obs = np.zeros(self.observation_space.shape, dtype=np.float32)
        return obs, 0.0, True, False, {}


@dataclass
class CorticalAgent:
    """High-level planner using PPO."""

    observation_space: Any
    action_space: Any
    model: PPO = field(init=False)

    def __post_init__(self) -> None:
        dummy_env = _StaticEnv(self.observation_space, self.action_space)
        self.model = PPO('MlpPolicy', dummy_env, verbose=0)

    def train(self, env, steps: int) -> None:
        self.model.set_env(env)
        self.model.learn(total_timesteps=steps)

    def infer(self, obs: np.ndarray) -> np.ndarray:
        action, _ = self.model.predict(obs, deterministic=True)
        return action

    def save(self, path: str) -> None:
        self.model.save(path)

    def load(self, path: str) -> None:
        self.model = PPO.load(path)
