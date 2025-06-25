from __future__ import annotations

import pybullet as p
import pybullet_data

from kinematics.kinematic_chain import KinematicChain


class PhysicsAgent:
    """Simple PyBullet wrapper for a kinematic chain."""

    def __init__(self, chain: KinematicChain) -> None:
        self.chain = chain
        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        self.bodies: dict[str, int] = {}
        self._build()

    def _build(self) -> None:
        p.loadURDF("plane.urdf")
        for uid, bone in self.chain.bones.items():
            length = bone.dimensions.get("length_cm", 1) / 100.0
            radius = bone.dimensions.get("width_cm", 1) / 200.0
            mass = bone.mass_kg() or 1.0
            col = p.createCollisionShape(p.GEOM_CAPSULE, radius=radius, height=length)
            vis = p.createVisualShape(p.GEOM_CAPSULE, radius=radius, length=length)
            body = p.createMultiBody(mass, col, vis, basePosition=[0, 0, 1])
            self.bodies[uid] = body

    def apply_joint_torque(self, uid: str, tau: float) -> None:
        body = self.bodies.get(uid)
        if body is not None:
            p.setJointMotorControlArray(body, [0], p.TORQUE_CONTROL, forces=[tau])

    def step(self, dt: float) -> None:
        p.setTimeStep(dt)
        p.stepSimulation()

    def get_joint_state(self, uid: str):
        body = self.bodies.get(uid)
        if body is None:
            return None
        return p.getBasePositionAndOrientation(body)

    def get_bone_force(self, uid: str):
        body = self.bodies.get(uid)
        if body is None:
            return None
        return p.getJointState(body, 0)[2]
