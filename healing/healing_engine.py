from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict

from damage.damage_engine import DamageEngine


@dataclass
class HealingEngine:
    damage: DamageEngine
    timers: Dict[str, float] = field(default_factory=dict)

    def update(self, dt: float) -> None:
        for uid, dmg in self.damage.damage.items():
            if dmg <= 0.0:
                self.timers[uid] = 0.0
                continue
            self.timers[uid] = self.timers.get(uid, 0.0) + dt
            # after 6 simulated weeks reduce damage substantially
            if self.timers[uid] >= 6 * 7 * 24 * 3600:
                self.damage.damage[uid] = max(0.0, dmg * 0.2)
