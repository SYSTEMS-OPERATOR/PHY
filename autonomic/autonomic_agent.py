from __future__ import annotations

from dataclasses import dataclass

from energy.energy_agent import EnergyAgent

@dataclass
class AutonomicAgent:
    """Regulate basic homeostatic outputs."""

    energy_agent: EnergyAgent
    core_temp_c: float = 37.0

    def update(self, dt: float) -> None:
        if self.core_temp_c > 37.4:
            self.energy_agent.kcal += 0.01 * dt
        elif self.core_temp_c < 36.3:
            self.energy_agent.kcal += 0.02 * dt
