from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable
import math

from .muscle_spec import MuscleSpec


@dataclass
class MuscleAgent:
    spec: MuscleSpec
    length_func: Callable[[], float] | None = None
    velocity_func: Callable[[], float] | None = None
    force_N: float = 0.0

    def attach_to_bones(self, length_cb: Callable[[], float], velocity_cb: Callable[[], float]) -> None:
        self.length_func = length_cb
        self.velocity_func = velocity_cb

    def update(self, dt: float, activation: float | None = None) -> float:
        if activation is not None:
            self.spec.activation = max(0.0, min(1.0, activation))
        length = self.length_func() if self.length_func else self.spec.optimal_fiber_len_cm
        vel = self.velocity_func() if self.velocity_func else 0.0
        # Simplified Hill-type: F = a * Fmax * (1 - ((l - l0)/l0)**2)
        l0 = self.spec.optimal_fiber_len_cm
        force = self.spec.activation * self.spec.max_isometric_force_N
        force *= max(0.0, 1.0 - ((length - l0) / l0) ** 2)
        # velocity modifier (very rough)
        force *= 1.0 - 0.1 * vel
        self.force_N = force
        return force

    def plot_force_length(self, num: int = 20) -> tuple[list[float], list[float]]:
        l0 = self.spec.optimal_fiber_len_cm
        ls = [l0 * (0.5 + i / num) for i in range(num + 1)]
        forces = []
        for l in ls:
            f = self.spec.max_isometric_force_N * max(0.0, 1.0 - ((l - l0) / l0) ** 2)
            forces.append(f)
        return ls, forces
