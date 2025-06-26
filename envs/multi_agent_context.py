from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple

import numpy as np
import gymnasium as gym

from envs.base_env import BaseEnv


@dataclass
class MultiAgentContext(BaseEnv):
    """Simple multi-agent extension of BaseEnv."""

    n_agents: int = 2
    states: Dict[str, np.ndarray] = field(default_factory=dict)

    def __post_init__(self) -> None:
        super().__post_init__()
        for i in range(self.n_agents):
            aid = f"agent_{i}"
            self.states[aid] = np.zeros_like(self.state)

    def reset(self, *, seed=None, options=None) -> Tuple[np.ndarray, dict]:
        obs, info = super().reset(seed=seed, options=options)
        for k in self.states:
            self.states[k][:] = 0
        return obs, info

    def step(self, actions):
        if isinstance(actions, dict):
            act0 = actions.get("agent_0", np.zeros_like(self.state))
        else:
            act0 = actions
            actions = {"agent_0": actions}
        obs, reward, done, trunc, info = super().step(act0)
        for k, act in actions.items():
            if k in self.states:
                self.states[k] += 0.1 * np.array(act)
        return obs, reward, done, trunc, info
