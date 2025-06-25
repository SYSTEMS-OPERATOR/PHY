from __future__ import annotations

from dataclasses import dataclass
from typing import Dict
import os
import tempfile
import pybullet as pb
import pybullet_data

from kinematics.kinematic_chain import KinematicChain
from export.urdf_exporter import export_chain_urdf


@dataclass
class PhysicsAgent:
    chain: KinematicChain

    def __post_init__(self) -> None:
        self.client = pb.connect(pb.DIRECT)
        pb.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        pb.setGravity(0, 0, -9.81, physicsClientId=self.client)
        fd, path = tempfile.mkstemp(suffix='.urdf')
        os.close(fd)
        export_chain_urdf(self.chain, path)
        self.robot = pb.loadURDF(path, basePosition=[0, 0, 1], physicsClientId=self.client)
        self.joint_map: Dict[str, int] = {}
        for i in range(pb.getNumJoints(self.robot, physicsClientId=self.client)):
            info = pb.getJointInfo(self.robot, i, physicsClientId=self.client)
            self.joint_map[info[1].decode()] = i
        self.path = path

    def apply_joint_torque(self, name: str, torque: float) -> None:
        jid = self.joint_map.get(name)
        if jid is not None:
            pb.setJointMotorControl2(self.robot, jid, pb.TORQUE_CONTROL, force=torque, physicsClientId=self.client)

    def step(self, dt: float) -> None:
        pb.setTimeStep(dt, physicsClientId=self.client)
        pb.stepSimulation(physicsClientId=self.client)

    def get_joint_state(self, name: str) -> float:
        jid = self.joint_map.get(name)
        if jid is None:
            return 0.0
        state = pb.getJointState(self.robot, jid, physicsClientId=self.client)
        return state[0]

    def get_bone_force(self, name: str) -> float:
        jid = self.joint_map.get(name)
        if jid is None:
            return 0.0
        state = pb.getJointState(self.robot, jid, physicsClientId=self.client)
        return state[2]

