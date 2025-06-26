from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np

from envs.multi_agent_context import MultiAgentContext


@dataclass
class TugOfWarEnv(MultiAgentContext):
    """Both agents pull a rope; reward is position delta."""

    position: float = 0.0

    def step(self, actions):
        obs, reward, done, trunc, info = super().step(actions)
        if isinstance(actions, dict):
            left = actions.get('agent_0', [0])[0]
            right = actions.get('agent_1', [0])[0]
        else:
            left = actions[0]
            right = 0.0
        self.position += 0.1 * (left - right)
        reward = -abs(self.position)
        if abs(self.position) < 0.1:
            done = True
        return obs, reward, done, trunc, info


@dataclass
class CubeCaptureEnv(MultiAgentContext):
    """Agents race to a cube in the middle."""

    cube: float = 0.0
    captured_by: str | None = None

    def step(self, actions):
        obs, reward, done, trunc, info = super().step(actions)
        if isinstance(actions, dict):
            pass
        else:
            actions = {"agent_0": actions}
        if self.captured_by is None:
            a0 = abs(self.states['agent_0'][0] - self.cube)
            a1 = abs(self.states['agent_1'][0] - self.cube)
            if a0 < 0.1 and a0 < a1:
                self.captured_by = 'agent_0'
            elif a1 < 0.1 and a1 < a0:
                self.captured_by = 'agent_1'
        if self.captured_by:
            reward = 1.0
            done = True
        else:
            reward = -0.01
        return obs, reward, done, trunc, info
