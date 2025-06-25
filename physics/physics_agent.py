from __future__ import annotations

from dataclasses import dataclass
from typing import Dict
import pybullet as p
import pybullet_data

from kinematics.kinematic_chain import KinematicChain
from geometry.geometry_agent import GeometryAgent


@dataclass
class PhysicsAgent:
    chain: KinematicChain

    def __post_init__(self) -> None:
        p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        self.joint_map: Dict[str, int] = {}
        self._build_multibody()

    def _build_multibody(self) -> None:
        base = self.chain.root
        base_mass = 1.0
        base_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05,0.05,0.05])
        base_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05,0.05,0.05])
        link_masses = [1.0 for _ in self.chain.joints]
        link_collisions = [p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02,0.02,0.02]) for _ in self.chain.joints]
        link_visuals = [p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02,0.02,0.02]) for _ in self.chain.joints]
        link_positions = [j.origin_xyz for j in self.chain.joints]
        link_orientations = [p.getQuaternionFromEuler(j.origin_rpy) for j in self.chain.joints]
        link_inertial = [(0,0,0)] * len(self.chain.joints)
        link_axes = [j.axis for j in self.chain.joints]
        joint_types = [p.JOINT_REVOLUTE if j.joint_type != 'fixed' else p.JOINT_FIXED for j in self.chain.joints]
        self.body = p.createMultiBody(
            base_mass,
            base_col,
            base_vis,
            basePosition=[0,0,1],
            linkMasses=link_masses,
            linkCollisionShapeIndices=link_collisions,
            linkVisualShapeIndices=link_visuals,
            linkPositions=link_positions,
            linkOrientations=link_orientations,
            linkInertialFramePositions=link_inertial,
            linkInertialFrameOrientations=[[0,0,0,1]]*len(self.chain.joints),
            linkParentIndices=[0]*len(self.chain.joints),
            linkJointTypes=joint_types,
            linkJointAxis=link_axes,
        )
        for idx, joint in enumerate(self.chain.joints):
            self.joint_map[joint.child_uid] = idx

    def apply_joint_torque(self, uid: str, tau: float) -> None:
        jid = self.joint_map.get(uid)
        if jid is not None:
            p.setJointMotorControl2(self.body, jid, p.TORQUE_CONTROL, force=tau)

    def step(self, dt: float) -> None:
        p.stepSimulation()

    def get_joint_state(self, uid: str):
        jid = self.joint_map.get(uid)
        if jid is None:
            return None
        return p.getJointState(self.body, jid)

    def get_bone_force(self, uid: str):
        jid = self.joint_map.get(uid)
        if jid is None:
            return None
        info = p.getJointState(self.body, jid)
        return info[2] if info else None

    def base_height(self) -> float:
        pos, _ = p.getBasePositionAndOrientation(self.body)
        return pos[2]

