from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict

@dataclass
class HealingEngine:
    """Simplified healing model restoring material strength over time."""

    bone: object
    modulus_key: str = "Youngs_modulus_GPa"
    recovery_time: float = 42.0  # days
    damage_time: float = 0.0
    start_modulus: float = 0.0
    state: str = "healthy"

    def start_healing(self, time: float) -> None:
        self.damage_time = time
        self.start_modulus = self.bone.material.get(self.modulus_key, 1.0) * 0.5
        self.bone.material[self.modulus_key] = self.start_modulus
        self.state = "healing"

    def update(self, t: float) -> None:
        if self.state != "healing":
            return
        elapsed = t - self.damage_time
        frac = min(1.0, elapsed / self.recovery_time)
        target = self.start_modulus + frac * (self.start_modulus)
        self.bone.material[self.modulus_key] = target
        if frac >= 1.0:
            self.state = "healthy"
