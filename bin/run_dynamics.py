#!/usr/bin/env python
from __future__ import annotations
import argparse
from skeleton.datasets import bones_from_dataset, load_dataset
from geometry.geometry_agent import GeometryAgent
from joints.joint_spec import hinge
from kinematics.kinematic_chain import KinematicChain
from physics.physics_agent import PhysicsAgent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', default='stand')
    parser.add_argument('--material', default='organic')
    args = parser.parse_args()

    bones = bones_from_dataset('female_21_baseline')[:2]
    dataset = load_dataset('female_21_baseline')
    for b in bones:
        b.set_material(args.material)
        GeometryAgent(b, dataset).compute()
    chain = KinematicChain({b.unique_id: b for b in bones}, [hinge('j1', bones[0].unique_id, bones[1].unique_id)], bones[0].unique_id)
    physics = PhysicsAgent(chain)

    if args.demo == 'drop':
        for _ in range(240*5):
            physics.step(1/240)
    else:
        physics.step(1/240)

if __name__ == '__main__':
    main()
