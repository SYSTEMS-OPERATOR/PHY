from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from envs.multi_agent_context import MultiAgentContext


@dataclass
class TugOfWarEnv(MultiAgentContext):
    """Both agents pull a rope; reward is position delta."""

    position: float = 0.0

    def step(self, actions: Any):
        obs, _, done, trunc, info = super().step(actions)
        if isinstance(actions, dict):
            left = float(actions.get("agent_0", [0.0])[0])
            right = float(actions.get("agent_1", [0.0])[0])
        else:
            left = float(actions[0])
            right = 0.0

        # Dev Agent Breadcrumb: position moves toward the stronger puller each frame.
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

    def step(self, actions: Any):
        obs, _, done, trunc, info = super().step(actions)
        if not isinstance(actions, dict):
            actions = {"agent_0": actions, "agent_1": [0.0]}

        # Dev Agent Breadcrumb: capture only occurs when one agent is clearly closest.
        if self.captured_by is None:
            dist_agent_0 = abs(self.states["agent_0"][0] - self.cube)
            dist_agent_1 = abs(self.states["agent_1"][0] - self.cube)
            if dist_agent_0 < 0.1 and dist_agent_0 < dist_agent_1:
                self.captured_by = "agent_0"
            elif dist_agent_1 < 0.1 and dist_agent_1 < dist_agent_0:
                self.captured_by = "agent_1"

        if self.captured_by:
            reward = 1.0
            done = True
            info["captured_by"] = self.captured_by
        else:
            reward = -0.01
        return obs, reward, done, trunc, info
