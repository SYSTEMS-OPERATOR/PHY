import pybullet as pb
from skeleton.base import BoneSpec
from geometry.geometry_agent import GeometryAgent
from joints.joint_spec import hinge
from kinematics.kinematic_chain import KinematicChain
from physics.physics_agent import PhysicsAgent


def build_chain():
    b1 = BoneSpec('A', 'long', {}, [], {'length_cm': 10.0, 'width_cm': 2.0, 'thickness_cm': 2.0}, [], [], '', '', 'A')
    b2 = BoneSpec('B', 'long', {}, [], {'length_cm': 10.0, 'width_cm': 2.0, 'thickness_cm': 2.0}, [], [], '', '', 'B')
    for b in (b1, b2):
        b.set_material('organic')
        b.set_embodiment('physical', b.material)
        GeometryAgent(b).compute()
    j = hinge('j1', 'A', 'B', axis=(1,0,0), limit=(-180,180), origin_xyz=(0,0,0.1))
    chain = KinematicChain({'A': b1, 'B': b2}, [j], 'A')
    return chain


def test_pybullet_energy_consistency():
    chain = build_chain()
    agent = PhysicsAgent(chain)
    start_z = pb.getBasePositionAndOrientation(agent.robot, physicsClientId=agent.client)[0][2]
    initial_potential = start_z * 9.81
    for _ in range(240):
        agent.step(1/240)
    mid_z = pb.getBasePositionAndOrientation(agent.robot, physicsClientId=agent.client)[0][2]
    assert mid_z < start_z

