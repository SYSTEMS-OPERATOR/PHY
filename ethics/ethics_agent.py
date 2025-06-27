"""Ethics agent performing risk-benefit analysis and event reporting."""
from __future__ import annotations

import datetime
from dataclasses import dataclass
from typing import List


@dataclass
class AdverseEvent:
    timestamp: datetime.datetime
    description: str


class EthicsAgent:
    def __init__(self, threshold: float = 0.3) -> None:
        self.threshold = threshold
        self.events: List[AdverseEvent] = []

    def assess_risk(self, risk: float, benefit: float) -> bool:
        ratio = risk / benefit if benefit else 1.0
        return ratio >= self.threshold

    def record_event(self, description: str) -> None:
        self.events.append(AdverseEvent(datetime.datetime.utcnow(), description))

    def generate_medwatch_xml(self) -> str:
        return "<medwatch><events>{}</events></medwatch>".format(len(self.events))
