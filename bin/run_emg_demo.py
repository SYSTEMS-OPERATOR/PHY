#!/usr/bin/env python
from __future__ import annotations

import argparse
import time

import numpy as np

from skeleton.bones import load_bones
from geometry.geometry_agent import GeometryAgent
from joints.joint_spec import hinge
from kinematics.kinematic_chain import KinematicChain
from physics.physics_agent import PhysicsAgent
from soft.muscle_spec import MuscleSpec
from soft.muscle_agent import MuscleAgent
from control.control_agent import ControlAgent
from energy.energy_agent import EnergyAgent


def build_chain() -> tuple[PhysicsAgent, MuscleAgent]:
    bones = load_bones("female_21_baseline")
    humerus = next(b for b in bones if b.unique_id == "BONE_HUMERUS_L")
    ulna = next(b for b in bones if b.unique_id == "BONE_ULNA_L")
    for b in (humerus, ulna):
        b.set_material("organic")
        b.set_embodiment("physical", b.material)
        GeometryAgent(b).compute()
    j = hinge("elbow", humerus.unique_id, ulna.unique_id, axis=(1, 0, 0), limit=(-180, 0), origin_xyz=(0, 0, humerus.dimensions.get("length_cm", 30)/100))
    chain = KinematicChain({humerus.unique_id: humerus, ulna.unique_id: ulna}, [j], humerus.unique_id)
    muscle = MuscleAgent(
        MuscleSpec("biceps", {"bone_uid": humerus.unique_id, "point": "distal"}, {"bone_uid": ulna.unique_id, "point": "proximal"}, 300.0, 10.0, 10.0),
        joint_name="elbow",
        moment_arm_m=0.03,
    )
    agent = PhysicsAgent(chain, muscles=[muscle], controller=None, energy=EnergyAgent())
    return agent, muscle


def run_demo(duration: float = 2.0) -> None:
    agent, muscle = build_chain()
    ctrl = ControlAgent([muscle.spec.name])
    agent.controller = ctrl
    t = 0.0
    while t < duration:
        emg = {muscle.spec.name: [0.5]}
        ctrl.update(1/240, emg)
        agent.step(1/240)
        t += 1/240
    angle = agent.get_joint_state("elbow")
    print(f"final elbow angle: {angle:.2f} deg")
    print(f"energy: {agent.energy.daily_report():.4f} kcal")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="EMG demo")
    parser.add_argument("--duration", type=float, default=2.0)
    args = parser.parse_args()
    run_demo(args.duration)
