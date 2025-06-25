from __future__ import annotations

from dataclasses import dataclass


@dataclass
class LigamentAgent:
    """Simple linear-viscoelastic ligament acting on a joint."""

    joint_name: str
    rest_angle_deg: float
    stiffness_Nm_deg: float
    damping_Nm_s_deg: float

    prev_angle_deg: float = 0.0

    def update(self, angle_deg: float, velocity_deg_s: float) -> float:
        delta = angle_deg - self.rest_angle_deg
        force = -self.stiffness_Nm_deg * delta - self.damping_Nm_s_deg * velocity_deg_s
        self.prev_angle_deg = angle_deg
        return force
