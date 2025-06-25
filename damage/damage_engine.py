from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

@dataclass
class DamageEvent:
    uid: str
    severity: float
    time: float

@dataclass
class DamageEngine:
    """Track strain energy and emit damage events."""

    yield_strength: float = 1.0
    ultimate_strength: float = 2.0
    events: List[DamageEvent] = field(default_factory=list)

    def accumulate(self, uid: str, load: float, t: float) -> None:
        if load > self.ultimate_strength:
            self.events.append(DamageEvent(uid, 1.0, t))
        elif load > self.yield_strength:
            self.events.append(DamageEvent(uid, 0.5, t))
