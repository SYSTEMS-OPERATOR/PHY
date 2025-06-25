from __future__ import annotations

import argparse

from skeleton.datasets import bones_from_dataset
from joints.joint_spec import hinge, fixed
from kinematics.kinematic_chain import KinematicChain
from physics.physics_agent import PhysicsAgent


def build_simple_chain():
    bones = bones_from_dataset('female_21_baseline')[:2]
    bones_dict = {b.unique_id: b for b in bones}
    j = hinge('joint1', bones[0].unique_id, bones[1].unique_id)
    chain = KinematicChain(bones[0], [j], bones_dict)
    return bones, chain


def demo_stand():
    bones, chain = build_simple_chain()
    phys = PhysicsAgent(chain)
    phys.build()
    for _ in range(240):
        phys.step(1/60)


def demo_drop(material: str):
    bones, chain = build_simple_chain()
    for b in bones:
        b.set_embodiment('physical', {'density': 4500} if material != 'organic' else {'density': 1800})
    phys = PhysicsAgent(chain)
    phys.build()
    for _ in range(240):
        phys.step(1/60)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', default='stand', choices=['stand','drop'])
    parser.add_argument('--material', default='organic')
    args = parser.parse_args()

    if args.demo == 'stand':
        demo_stand()
    else:
        demo_drop(args.material)


if __name__ == '__main__':
    main()
