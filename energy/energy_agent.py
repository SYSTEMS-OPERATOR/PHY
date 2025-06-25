from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class EnergyAgent:
    kcal: float = 0.0
    efficiency: float = 0.25

    def accumulate(self, force_N: float, velocity_m_s: float, dt: float) -> None:
        work = force_N * velocity_m_s * dt
        self.kcal += work / self.efficiency / 4184

    def daily_report(self) -> float:
        return self.kcal
