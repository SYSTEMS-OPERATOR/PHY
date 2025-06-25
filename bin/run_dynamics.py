from __future__ import annotations

import argparse

from skeleton.datasets import bones_from_dataset
from geometry.geometry_agent import GeometryAgent
from joints.joint_spec import JointSpec
from kinematics.kinematic_chain import KinematicChain
from physics.physics_agent import PhysicsAgent


def build_simple_chain(material: str = "organic") -> PhysicsAgent:
    bones = bones_from_dataset("female_21_baseline")[:2]
    for b in bones:
        b.set_material(material)
        b.set_embodiment("physical", b.material)
        GeometryAgent(b).compute()
    j = JointSpec.hinge("joint0", bones[0].unique_id, bones[1].unique_id)
    chain = KinematicChain({b.unique_id: b for b in bones}, [j])
    return PhysicsAgent(chain)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run dynamics demo")
    parser.add_argument("--demo", choices=["stand", "drop"], default="stand")
    parser.add_argument("--material", default="organic")
    args = parser.parse_args()
    physics = build_simple_chain(args.material)
    if args.demo == "drop":
        steps = int(5 / 0.01)
        for _ in range(steps):
            physics.step(0.01)
    print("simulation complete")


if __name__ == "__main__":
    main()
