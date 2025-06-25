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
    """Detect simple bone damage from stress."""

    bones: Dict[str, "BoneSpec"]
    events: List[DamageEvent] = field(default_factory=list)
    time: float = 0.0

    def accumulate(self, loads: Dict[str, float], dt: float) -> None:
        self.time += dt
        for uid, stress in loads.items():
            bone = self.bones.get(uid)
            if bone is None:
                continue
            yield_strength = bone.material.get("yield_strength", 1e6)
            ult_strength = bone.material.get("ultimate_strength", 2e6)
            if stress > yield_strength:
                severity = 0.1 if stress < ult_strength else 1.0
                self.events.append(DamageEvent(uid, severity, self.time))
