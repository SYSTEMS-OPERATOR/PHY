from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np

from envs.multi_agent_context import MultiAgentContext


@dataclass
class PassTheCubeEnv(MultiAgentContext):
    """Toy environment where agent_0 hands cube to agent_1."""

    cube_pos: float = 0.0
    threshold: float = 0.5

    def reset(self, *, seed=None, options=None) -> Tuple[np.ndarray, dict]:
        obs, info = super().reset(seed=seed, options=options)
        self.cube_pos = 0.0
        return obs, info

    def step(self, actions):
        obs, reward, done, trunc, info = super().step(actions)
        a0 = self.states["agent_0"][0]
        a1 = self.states["agent_1"][0]
        if abs(a0 - a1) < self.threshold:
            self.cube_pos = a1
            reward = 1.0
            done = True
        else:
            reward = -0.01
        return obs, reward, done, trunc, info


@dataclass
class TeamRampEnv(MultiAgentContext):
    """Two agents carry a plank up an incline."""

    incline: float = 0.1
    reached: bool = False

    def step(self, actions):
        obs, reward, done, trunc, info = super().step(actions)
        avg = (self.states['agent_0'][0] + self.states['agent_1'][0]) / 2
        if avg > self.incline:
            self.reached = True
            reward = 1.0
            done = True
        else:
            reward = -0.01
        return obs, reward, done, trunc, info


@dataclass
class TrustFallEnv(MultiAgentContext):
    """Agent_0 falls, agent_1 must catch."""

    falling: bool = False

    def step(self, actions):
        obs, reward, done, trunc, info = super().step(actions)
        if not self.falling:
            self.falling = True
        catcher = self.states['agent_1'][0]
        if self.falling and abs(catcher) < 0.2:
            reward = 1.0
            done = True
        else:
            reward = -0.01
        return obs, reward, done, trunc, info
