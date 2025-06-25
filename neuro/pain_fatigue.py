from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PainFatigueModel:
    """Map nociceptor firing and metabolic fatigue to activation scaling."""

    pain: float = 0.0  # range 0-10
    fatigue: float = 0.0  # range 0-1
    ignore_pain: bool = False
    ignore_fatigue: bool = False

    def update_pain(self, firing: float) -> None:
        self.pain = max(self.pain, min(10.0, firing))

    def update_fatigue(self, val: float) -> None:
        self.fatigue = max(0.0, min(1.0, val))

    def apply(self, activation_cmd: float) -> float:
        scale = 1.0
        if not self.ignore_pain:
            scale *= max(0.0, 1.0 - 0.1 * self.pain)
        if not self.ignore_fatigue:
            scale *= max(0.0, 1.0 - self.fatigue)
        return activation_cmd * scale
