from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List


@dataclass
class MentalState:
    belief: List[float] = field(default_factory=list)
    goal: List[float] = field(default_factory=list)
    emotion: float = 0.0


@dataclass
class SocialCognitionAgent:
    """Track simple mental state of other agents."""

    others: Dict[str, MentalState] = field(default_factory=dict)

    def update(self, agent_id: str, belief: List[float]) -> None:
        st = self.others.setdefault(agent_id, MentalState())
        st.belief = belief

    def predict_actions(self, agent_id: str, horizon: int = 1) -> List[List[float]]:
        st = self.others.get(agent_id, MentalState())
        return [st.goal for _ in range(horizon)]
