from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from .muscle_spec import MuscleSpec


@dataclass
class MuscleAgent:
    """Minimal Hill-type muscle model producing joint torque."""

    spec: MuscleSpec
    joint_name: str
    moment_arm_m: float

    length_m: float = 0.0
    velocity_m_s: float = 0.0
    last_torque: float = 0.0

    def update(self, dt: float, activation: Optional[float] = None) -> float:
        """Return torque to apply at the joint."""
        if activation is None:
            activation = self.spec.activation
        self.spec.activation = max(0.0, min(1.0, activation))
        f_iso = self.spec.max_isometric_force_N
        force = self.spec.activation * f_iso
        torque = force * self.moment_arm_m
        self.last_torque = torque
        return torque
