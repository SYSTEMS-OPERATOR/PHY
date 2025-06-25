from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from .receptor_spec import ReceptorSpec


@dataclass
class SensorAgent:
    """Simplified proprioceptive/nociceptive sensor."""

    spec: ReceptorSpec
    muscle: Optional["MuscleAgent"] = None
    physics: Optional["PhysicsAgent"] = None
    joint_name: Optional[str] = None
    firing_hz: float = 0.0
    prev_angle: float = 0.0

    def update(self, dt: float) -> float:
        """Update firing rate based on stretch, tension or damage."""
        signal = 0.0
        if self.spec.type == "muscle_spindle" and self.joint_name and self.physics:
            angle = self.physics.get_joint_state(self.joint_name)
            vel = (angle - self.prev_angle) / dt
            self.prev_angle = angle
            strain = abs(vel)
            signal = max(0.0, strain - self.spec.threshold) * self.spec.signal_gain
        elif self.spec.type == "GTO" and self.muscle is not None:
            tension = getattr(self.muscle, "last_torque", 0.0) / max(self.muscle.moment_arm_m, 1e-6)
            signal = max(0.0, tension - self.spec.threshold) * self.spec.signal_gain
        elif self.spec.type == "nociceptor" and self.muscle is not None:
            dmg = getattr(self.muscle, "damage_ratio", 0.0)
            signal = dmg * self.spec.signal_gain
        self.firing_hz = signal
        return self.firing_hz
