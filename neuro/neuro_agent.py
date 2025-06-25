from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List

from sensors.sensor_agent import SensorAgent

@dataclass
class NeuroAgent:
    """Integrate afferent signals and apply simple reflex arcs."""

    muscles: Dict[str, object]
    sensors: Iterable[SensorAgent]
    queue: Dict[str, List[float]] = field(default_factory=dict)

    def step(self, dt: float) -> None:
        for s in self.sensors:
            rate = s.update(dt)
            self.queue.setdefault(s.spec.name, []).append(rate)
            if s.spec.type == "muscle_spindle" and rate > 0:
                m = self.muscles.get(s.spec.name)
                if m:
                    m.spec.activation = min(1.0, m.spec.activation + 0.1 * rate * dt)
            if s.spec.type == "nociceptor" and rate > 0:
                for m in self.muscles.values():
                    if "flexor" in m.spec.name:
                        m.spec.activation = 1.0
                    if "extensor" in m.spec.name:
                        m.spec.activation = 0.0
