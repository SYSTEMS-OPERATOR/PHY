from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable

from sensors.sensor_agent import SensorAgent
from .pain_fatigue import PainFatigueModel


@dataclass
class NeuroAgent:
    """Integrate afferent signals and generate reflex activations."""

    sensors: SensorAgent
    pain_model: PainFatigueModel = field(default_factory=PainFatigueModel)
    withdrawal_flexors: Iterable[str] = field(default_factory=list)
    withdrawal_extensors: Iterable[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.reflex: Dict[str, float] = {}

    def step(self, dt: float) -> Dict[str, float]:
        self.reflex.clear()
        rates = self.sensors.firing
        for name, rate in rates.items():
            spec = self.sensors.receptors[[r.name for r in self.sensors.receptors].index(name)]
            if spec.type == "muscle_spindle" and rate > 0:
                mname = spec.location.get("muscle")
                self.reflex[mname] = self.reflex.get(mname, 0.0) + 0.5 * rate * dt
            elif spec.type == "GTO" and rate > 0:
                mname = spec.location.get("muscle")
                self.reflex[mname] = self.reflex.get(mname, 0.0) - 0.5 * rate * dt
            elif spec.type == "nociceptor" and rate > 0:
                self.pain_model.update_pain(rate)
                for m in self.withdrawal_flexors:
                    self.reflex[m] = self.reflex.get(m, 0.0) + 1.0
                for m in self.withdrawal_extensors:
                    self.reflex[m] = self.reflex.get(m, 0.0) - 1.0
        return dict(self.reflex)
