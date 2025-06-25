from __future__ import annotations

from dataclasses import dataclass


@dataclass
class EnergyAgent:
    """Simple metabolic energy tracker."""

    kcal: float = 0.0
    efficiency: float = 0.25

    def accumulate(self, torque: float, angular_velocity_rad_s: float, dt: float) -> None:
        work = torque * angular_velocity_rad_s * dt
        if self.efficiency > 0:
            self.kcal += work / self.efficiency / 4184

    def daily_report(self) -> float:
        return self.kcal
