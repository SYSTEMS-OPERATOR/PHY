from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List

from sensors.sensor_agent import SensorAgent


@dataclass
class NeuroAgent:
    """Integrate afferent firing and apply spinal reflexes."""

    sensors: List[SensorAgent] = field(default_factory=list)
    reflex_commands: Dict[str, float] = field(default_factory=dict)

    def stretch_reflex(self) -> None:
        for s in self.sensors:
            if s.spec.type == "muscle_spindle" and s.muscle is not None:
                if s.firing_hz > 0.0:
                    self.reflex_commands[s.muscle.spec.name] = self.reflex_commands.get(s.muscle.spec.name, 0.0) + 0.1 * s.firing_hz

    def gto_inhibition(self) -> None:
        for s in self.sensors:
            if s.spec.type == "GTO" and s.muscle is not None:
                if s.firing_hz > 0.0:
                    self.reflex_commands[s.muscle.spec.name] = self.reflex_commands.get(s.muscle.spec.name, 0.0) - 0.1 * s.firing_hz

    def withdrawal_reflex(self) -> None:
        for s in self.sensors:
            if s.spec.type == "nociceptor" and s.muscle is not None:
                if s.firing_hz > 0.0:
                    self.reflex_commands[s.muscle.spec.name] = self.reflex_commands.get(s.muscle.spec.name, 0.0) + 0.2 * s.firing_hz

    def step(self, dt: float) -> Dict[str, float]:
        self.reflex_commands.clear()
        for s in self.sensors:
            s.update(dt)
        self.stretch_reflex()
        self.gto_inhibition()
        self.withdrawal_reflex()
        return dict(self.reflex_commands)
