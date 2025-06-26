from __future__ import annotations

from dataclasses import dataclass
import numpy as np
import gymnasium as gym


@dataclass
class BaseEnv(gym.Env):
    """Minimal gym-compatible environment."""

    def __post_init__(self) -> None:
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(4,), dtype=np.float32)
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)
        self.state = np.zeros(4, dtype=np.float32)
        self.t = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.state[:] = 0
        self.t = 0
        return self.state, {}

    def step(self, action):
        self.state = self.state + 0.1 * np.array(action)
        self.t += 1
        reward = float(-np.sum(self.state ** 2))
        done = self.t >= 100
        return self.state, reward, done, False, {}
