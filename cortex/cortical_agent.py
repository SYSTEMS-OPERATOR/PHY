from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.envs import DummyEnv


@dataclass
class CorticalAgent:
    """High-level planner using PPO."""

    observation_space: Any
    action_space: Any
    model: PPO = field(init=False)

    def __post_init__(self) -> None:
        self.model = PPO('MlpPolicy', DummyEnv(self.observation_space, self.action_space), verbose=0)

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
