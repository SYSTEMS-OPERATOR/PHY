from __future__ import annotations

from dataclasses import dataclass

from energy.energy_agent import EnergyAgent


@dataclass
class AutonomicAgent:
    energy: EnergyAgent
    core_temp: float = 37.0
    sweat: bool = False
    shiver: bool = False

    def update(self, dt: float) -> None:
        if self.core_temp > 37.4:
            self.sweat = True
        else:
            self.sweat = False
        if self.core_temp < 36.3:
            self.shiver = True
        else:
            self.shiver = False
