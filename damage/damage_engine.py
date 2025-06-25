from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List

from skeleton.base import BoneSpec


@dataclass
class DamageEvent:
    uid: str
    severity: float
    time: float


@dataclass
class DamageEngine:
    bones: Dict[str, BoneSpec]
    damage: Dict[str, float] = field(default_factory=dict)
    events: List[DamageEvent] = field(default_factory=list)

    def accumulate(self, loads: Dict[str, float], dt: float) -> None:
        for uid, load in loads.items():
            bone = self.bones.get(uid)
            if bone is None:
                continue
            yield_lim = bone.material.get("compressive_strength_MPa", 100.0)
            strain = abs(load) / max(yield_lim, 1.0)
            cur = self.damage.get(uid, 0.0)
            if strain > 1.0:
                cur = min(1.0, cur + 0.1 * strain * dt)
                self.events.append(DamageEvent(uid, cur, time.time()))
            elif strain > 0.5:
                cur = min(1.0, cur + 0.01 * strain * dt)
            self.damage[uid] = cur
            bone.damage = cur
