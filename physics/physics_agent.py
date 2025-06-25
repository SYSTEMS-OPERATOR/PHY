from __future__ import annotations

try:
    import pybullet as p  # type: ignore
    import pybullet_data
except Exception:  # pragma: no cover - optional dependency
    p = None
    pybullet_data = None
from typing import Dict

from kinematics.kinematic_chain import KinematicChain


class PhysicsAgent:
    def __init__(self, chain: KinematicChain) -> None:
        self.chain = chain
        if p is None:
            raise RuntimeError("pybullet not available")
        self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81, physicsClientId=self.client)
        self.bodies: Dict[str, int] = {}

    def build(self) -> None:
        for uid, (R, t) in self.chain.forward_kinematics({}).items():
            mass = 1.0
            shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01,0.01,0.01])
            body = p.createMultiBody(mass, shape)
            p.resetBasePositionAndOrientation(body, t, [0,0,0,1])
            self.bodies[uid] = body

    def apply_joint_torque(self, uid: str, tau: float) -> None:
        body = self.bodies.get(uid)
        if body is not None:
            p.applyExternalTorque(body, -1, [0,0,tau], p.WORLD_FRAME)

    def step(self, dt: float) -> None:
        p.stepSimulation()

    def get_joint_state(self, uid: str):
        body = self.bodies.get(uid)
        if body is None:
            return None
        pos, orn = p.getBasePositionAndOrientation(body)
        return pos, orn

    def get_bone_force(self, uid: str):
        body = self.bodies.get(uid)
        if body is None:
            return (0,0,0)
        force = p.getJointState(body, 0)
        return force
