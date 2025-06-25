from __future__ import annotations

from dataclasses import dataclass

@dataclass
class PainFatigueModel:
    """Attenuate voluntary activation based on pain or fatigue."""

    pain_level: float = 0.0
    fatigue: float = 0.0
    ignore_pain: bool = False
    ignore_fatigue: bool = False

    def activation_efficiency(self) -> float:
        pain_factor = 1.0 - 0.05 * self.pain_level if not self.ignore_pain else 1.0
        fatigue_factor = 1.0 - 0.5 * self.fatigue if not self.ignore_fatigue else 1.0
        return max(0.0, pain_factor * fatigue_factor)
