#!/usr/bin/env python
from __future__ import annotations

import argparse

from skeleton.bones import load_bones
from geometry.geometry_agent import GeometryAgent
from joints.joint_spec import hinge
from kinematics.kinematic_chain import KinematicChain
from physics.physics_agent import PhysicsAgent
from adaptation.wolff_engine import WolffAdaptationEngine


def build_agent() -> PhysicsAgent:
    bones = load_bones("female_21_baseline")
    femur = next(b for b in bones if b.unique_id == "BONE_FEMUR_L")
    tibia = next(b for b in bones if b.unique_id == "BONE_TIBIA_L")
    for b in (femur, tibia):
        b.set_material("organic")
        b.set_embodiment("physical", b.material)
        GeometryAgent(b).compute()
    j = hinge("knee", femur.unique_id, tibia.unique_id, axis=(1, 0, 0), limit=(-180, 0), origin_xyz=(0, 0, femur.dimensions.get("length_cm", 40)/100))
    chain = KinematicChain({femur.unique_id: femur, tibia.unique_id: tibia}, [j], femur.unique_id)
    wolff = WolffAdaptationEngine({femur.unique_id: femur, tibia.unique_id: tibia})
    agent = PhysicsAgent(chain, wolff=wolff)
    return agent


def run_demo(cycles: int = 10000) -> None:
    agent = build_agent()
    for _ in range(cycles):
        agent.apply_joint_torque("knee", -50.0)
        agent.step(1/240)
    femur = agent.chain.bones[next(iter(agent.chain.bones))]
    print(f"femur density: {femur.material['density']:.2f} kg/m^3")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Adaptation demo")
    parser.add_argument("--cycles", type=int, default=10000)
    args = parser.parse_args()
    run_demo(args.cycles)
