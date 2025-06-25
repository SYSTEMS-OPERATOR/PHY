import numpy as np
from skeleton.base import BoneSpec
from joints.joint_spec import hinge
from kinematics.kinematic_chain import KinematicChain
from physics.physics_agent import PhysicsAgent


def test_pybullet_energy_consistency():
    b1 = BoneSpec('b1','long',{},[],{'length_cm':1,'width_cm':1,'thickness_cm':1},[],[],"","",'B1')
    b2 = BoneSpec('b2','long',{},[],{'length_cm':1,'width_cm':1,'thickness_cm':1},[],[],"","",'B2')
    bones = {b1.unique_id: b1, b2.unique_id: b2}
    joints = [hinge('j1', b1.unique_id, b2.unique_id)]
    chain = KinematicChain(bones, joints, b1.unique_id)
    agent = PhysicsAgent(chain)
    heights = []
    for _ in range(240):
        agent.step(1/240)
        heights.append(agent.base_height())
    assert heights[0] > heights[-1]
