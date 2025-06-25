from __future__ import annotations

from dataclasses import dataclass


@dataclass
class AutonomicAgent:
    """Very small autonomic regulation placeholder."""

    energy_agent: "EnergyAgent"
    core_temp_c: float = 37.0
    sweat: bool = False
    shiver: bool = False

    def update(self, dt: float) -> None:
        if self.core_temp_c > 37.4:
            self.sweat = True
            self.energy_agent.kcal += 0.1 * dt
        else:
            self.sweat = False
        if self.core_temp_c < 36.3:
            self.shiver = True
            self.energy_agent.kcal += 0.2 * dt
        else:
            self.shiver = False
        deviation = self.core_temp_c - 37.0
        self.energy_agent.kcal += abs(deviation) * 0.05 * dt
