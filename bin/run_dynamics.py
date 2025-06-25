#!/usr/bin/env python
from __future__ import annotations

import argparse

from skeleton.bones import load_bones
from geometry.geometry_agent import GeometryAgent
from joints import joint_spec
from kinematics.kinematic_chain import KinematicChain
from physics.physics_agent import PhysicsAgent


def build_chain(material: str = "organic") -> KinematicChain:
    """Return a simple femur-tibia kinematic chain."""
    bones = load_bones("female_21_baseline")
    femur = next(b for b in bones if b.unique_id == "BONE_FEMUR_L")
    tibia = next(b for b in bones if b.unique_id == "BONE_TIBIA_L")

    for b in (femur, tibia):
        b.set_material(material)
        b.set_embodiment("physical", b.material)
        GeometryAgent(b).compute()

    origin_z = femur.dimensions.get("length_cm", 1.0) / 100
    j = joint_spec.hinge(
        "joint1",
        femur.unique_id,
        tibia.unique_id,
        axis=(0, 0, 1),
        limit=(-180, 180),
        origin_xyz=(0, 0, origin_z),
    )

    mapping = {femur.unique_id: femur, tibia.unique_id: tibia}
    chain = KinematicChain(mapping, [j], femur.unique_id)
    return chain


def demo_stand(material: str) -> None:
    chain = build_chain(material)
    PhysicsAgent(chain)


def demo_drop(material: str) -> None:
    chain = build_chain(material)
    agent = PhysicsAgent(chain)
    for _ in range(240*5):
        agent.step(1/240)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run dynamics demos")
    parser.add_argument("--demo", choices=["stand", "drop"], default="stand")
    parser.add_argument("--material", default="organic")
    args = parser.parse_args()
    if args.demo == "stand":
        demo_stand(args.material)
    else:
        demo_drop(args.material)


if __name__ == "__main__":
    main()

