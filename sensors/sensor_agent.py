from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .receptor_spec import ReceptorSpec


@dataclass
class SensorAgent:
    """Agent representing a single receptor."""

    spec: ReceptorSpec
    muscle: Optional[object] = None
    ligament: Optional[object] = None
    bone: Optional[object] = None
    firing_rate_hz: float = 0.0

    def update(self, dt: float) -> float:
        """Update firing rate based on linked tissue state."""
        strain = 0.0
        tension = 0.0
        if self.muscle is not None:
            strain = getattr(self.muscle, "length_m", 0.0)
            tension = getattr(self.muscle, "spec", {}).max_isometric_force_N * getattr(self.muscle.spec, "activation", 0.0)
        if self.ligament is not None:
            strain = getattr(self.ligament, "prev_angle_deg", 0.0)
        if self.bone is not None:
            strain = sum(self.bone.load)
        if self.spec.type in {"muscle_spindle", "GTO", "joint_capsule"}:
            self.firing_rate_hz = max(0.0, self.spec.signal_gain * (strain - self.spec.threshold))
        else:
            ratio = max(0.0, tension - self.spec.threshold)
            self.firing_rate_hz = self.spec.signal_gain * ratio
        return self.firing_rate_hz
