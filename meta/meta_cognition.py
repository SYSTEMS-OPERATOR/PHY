from __future__ import annotations

"""Self-monitoring meta-cognitive agent."""

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict


@dataclass
class MetaCognitionAgent:
    """Track training metrics and detect anomalies."""

    reward_window: int = 100
    reward_threshold: float = 1e-3
    infraction_limit: int = 5
    novelty_threshold: float = 5.0

    rewards: Deque[float] = field(default_factory=lambda: deque(maxlen=100))
    infractions: int = 0
    plan: Dict[str, str] | None = None

    def record_step(self, reward: float, safety_infraction: bool = False, novelty: float = 0.0) -> None:
        self.rewards.append(reward)
        if safety_infraction:
            self.infractions += 1
        if novelty > self.novelty_threshold:
            self.plan = {"action": "alert_human", "reason": "novelty spike"}

    def check_anomaly(self) -> Dict[str, str] | None:
        if len(self.rewards) >= self.reward_window:
            avg = sum(self.rewards) / len(self.rewards)
            if avg < self.reward_threshold:
                self.plan = {"action": "rollback", "reason": "reward collapse"}
        if self.infractions > self.infraction_limit:
            self.plan = {"action": "shrink_lr", "reason": "safety infractions"}
        return self.plan
