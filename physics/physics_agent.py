from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Iterable, Optional
import os
import tempfile
import pybullet as pb
import pybullet_data

from kinematics.kinematic_chain import KinematicChain
from export.urdf_exporter import export_chain_urdf
from soft.muscle_agent import MuscleAgent
from soft.ligament_agent import LigamentAgent
from control.control_agent import ControlAgent
from adaptation.wolff_engine import WolffAdaptationEngine
from energy.energy_agent import EnergyAgent
from sensors.sensor_agent import SensorAgent
from neuro.neuro_agent import NeuroAgent
from damage.damage_engine import DamageEngine
from healing.healing_engine import HealingEngine
from autonomic.autonomic_agent import AutonomicAgent


@dataclass
class PhysicsAgent:
    chain: KinematicChain
    muscles: List[MuscleAgent] = field(default_factory=list)
    ligaments: List[LigamentAgent] = field(default_factory=list)
    controller: Optional[ControlAgent] = None
    wolff: Optional[WolffAdaptationEngine] = None
    energy: Optional[EnergyAgent] = None
    sensors: Iterable[SensorAgent] = field(default_factory=list)
    neuro: Optional[NeuroAgent] = None
    damage_engine: Optional[DamageEngine] = None
    healing: Optional[HealingEngine] = None
    autonomic: Optional[AutonomicAgent] = None

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
            pb.setJointMotorControl2(self.robot, i, pb.VELOCITY_CONTROL, force=0, physicsClientId=self.client)
        self.path = path

    def apply_joint_torque(self, name: str, torque: float) -> None:
        jid = self.joint_map.get(name)
        if jid is not None:
            pb.setJointMotorControl2(self.robot, jid, pb.TORQUE_CONTROL, force=torque, physicsClientId=self.client)

    def step(self, dt: float) -> None:
        if self.neuro is not None:
            self.neuro.step(dt)
        activations: Dict[str, float] = {}
        if self.controller is not None:
            activations = self.controller.update(dt, {})
        for m in self.muscles:
            act = activations.get(m.spec.name, m.spec.activation)
            torque = m.update(dt, act)
            self.apply_joint_torque(m.joint_name, torque)
            if self.energy is not None:
                vel = self.get_joint_velocity(m.joint_name)
                self.energy.accumulate(torque, vel, dt)
            if self.damage_engine is not None:
                self.damage_engine.accumulate(m.spec.name, abs(torque), 0.0)
        for lig in self.ligaments:
            ang = self.get_joint_state(lig.joint_name)
            vel = self.get_joint_velocity(lig.joint_name)
            torque = lig.update(ang, vel)
            self.apply_joint_torque(lig.joint_name, torque)
        pb.setTimeStep(dt, physicsClientId=self.client)
        pb.stepSimulation(physicsClientId=self.client)
        if self.wolff is not None:
            loads = {uid: self.get_bone_force(uid) for uid in self.chain.bones}
            self.wolff.record(loads)
        if self.healing is not None:
            self.healing.update(0.0)
        if self.autonomic is not None:
            self.autonomic.update(dt)
        for s in self.sensors:
            s.update(dt)

    def get_joint_state(self, name: str) -> float:
        jid = self.joint_map.get(name)
        if jid is None:
            return 0.0
        state = pb.getJointState(self.robot, jid, physicsClientId=self.client)
        return state[0]

    def get_joint_velocity(self, name: str) -> float:
        jid = self.joint_map.get(name)
        if jid is None:
            return 0.0
        state = pb.getJointState(self.robot, jid, physicsClientId=self.client)
        return state[1]

    def get_bone_force(self, name: str) -> float:
        jid = self.joint_map.get(name)
        if jid is None:
            return 0.0
        state = pb.getJointState(self.robot, jid, physicsClientId=self.client)
        return state[2]

