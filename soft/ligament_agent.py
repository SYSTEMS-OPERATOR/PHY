from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


@dataclass
class LigamentSpec:
    name: str
    origin: dict
    insertion: dict
    stiffness_N_per_m: float
    damping_Ns_per_m: float


@dataclass
class LigamentAgent:
    spec: LigamentSpec
    length_func: Callable[[], float] | None = None
    velocity_func: Callable[[], float] | None = None

    def attach_to_bones(self, length_cb: Callable[[], float], velocity_cb: Callable[[], float]) -> None:
        self.length_func = length_cb
        self.velocity_func = velocity_cb

    def update(self, dt: float) -> float:
        length = self.length_func() if self.length_func else 0.0
        vel = self.velocity_func() if self.velocity_func else 0.0
        stretch = max(0.0, length)
        force = self.spec.stiffness_N_per_m * stretch + self.spec.damping_Ns_per_m * vel
        return force
