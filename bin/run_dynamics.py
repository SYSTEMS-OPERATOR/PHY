#!/usr/bin/env python
from __future__ import annotations

import argparse

from skeleton.bones import load_bones
from skeleton.field import SkeletonField
from skeleton.datasets import load_dataset
from geometry.geometry_agent import GeometryAgent
from joints import joint_spec
from kinematics.kinematic_chain import KinematicChain
from physics.physics_agent import PhysicsAgent


def build_chain(material: str = "organic") -> KinematicChain:
    bones = load_bones("female_21_baseline")[:2]
    dataset = load_dataset("female_21_baseline")
    for b in bones:
        b.set_material(material)
        b.set_embodiment("physical", b.material)
        GeometryAgent(b).compute()
    j = joint_spec.hinge("joint1", bones[0].unique_id, bones[1].unique_id, axis=(0,0,1), limit=(-180,180), origin_xyz=(0,0,bones[0].dimensions.get('length_cm',1)/100))
    mapping = {b.unique_id: b for b in bones}
    chain = KinematicChain(mapping, [j], bones[0].unique_id)
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

