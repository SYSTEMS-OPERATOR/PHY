from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import pybullet as pb

from physics.physics_agent import PhysicsAgent


@dataclass
class VestibularAgent:
    """Head motion sensor using PhysicsAgent."""

    physics: PhysicsAgent
    link_name: str = "head"
    noise_std: float = 0.01

    def read(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return angular velocity and linear acceleration."""
        jid = self.physics.joint_map.get(self.link_name)
        if jid is None:
            return np.zeros(3), np.zeros(3)
        vel = pb.getJointState(self.physics.robot, jid, physicsClientId=self.physics.client)[1]
        lin_acc = np.array(pb.getBaseVelocity(self.physics.robot, physicsClientId=self.physics.client)[1])
        vel = np.array([0.0, 0.0, vel])  # 1D joint
        vel += np.random.normal(scale=self.noise_std, size=3)
        lin_acc += np.random.normal(scale=self.noise_std, size=3)
        return vel, lin_acc
