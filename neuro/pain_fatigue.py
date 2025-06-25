from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PainFatigueModel:
    """Map nociceptor activity and fatigue to activation efficiency."""

    pain: float = 0.0  # 0-10 scale
    fatigue: float = 0.0  # 0-1 scale

    def activation_efficiency(self, cmd: float, ignore_pain: bool = False, ignore_fatigue: bool = False) -> float:
        pain_factor = 1.0
        if not ignore_pain:
            pain_factor = max(0.0, 1.0 - 0.05 * self.pain)
        fat_factor = 1.0
        if not ignore_fatigue:
            fat_factor = max(0.0, 1.0 - 0.5 * self.fatigue)
        return cmd * pain_factor * fat_factor
