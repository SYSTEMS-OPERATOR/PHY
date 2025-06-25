import numpy as np
from skeleton.base import BoneSpec
from geometry.geometry_agent import GeometryAgent
from joints.joint_spec import hinge
from kinematics.kinematic_chain import KinematicChain


def build_chain():
    b1 = BoneSpec('A', 'long', {}, [], {'length_cm': 10.0, 'width_cm': 2.0, 'thickness_cm': 2.0}, [], [], '', '', 'A')
    b2 = BoneSpec('B', 'long', {}, [], {'length_cm': 10.0, 'width_cm': 2.0, 'thickness_cm': 2.0}, [], [], '', '', 'B')
    for b in (b1, b2):
        b.set_material('organic')
        b.set_embodiment('physical', b.material)
        GeometryAgent(b).compute()
    j = hinge('j1', 'A', 'B', axis=(1,0,0), limit=(-180,180), origin_xyz=(0,0,0.1))
    return KinematicChain({'A': b1, 'B': b2}, [j], 'A')


def test_ik_reaches_target():
    chain = build_chain()
    target = np.array([0.0, 0.0, 0.2])
    sol = chain.inverse_kinematics('B', target)
    pos = chain.end_effector_position(sol, 'B')
    assert np.linalg.norm(pos - target) < 0.02

