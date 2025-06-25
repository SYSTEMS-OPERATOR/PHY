from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable


@dataclass
class ControlAgent:
    muscle_map: Dict[str, object]
    activations: Dict[str, float] = field(default_factory=dict)
    alpha: float = 0.1  # simple low-pass

    def update(self, dt: float, emg_signals: Iterable[float]) -> None:
        for (name, muscle), emg in zip(self.muscle_map.items(), emg_signals):
            act_prev = self.activations.get(name, 0.0)
            act = act_prev + self.alpha * (abs(emg) - act_prev)
            self.activations[name] = act
            muscle.update(dt, act)
