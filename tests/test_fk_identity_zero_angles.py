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


def test_fk_identity_zero_angles():
    chain = build_chain()
    tf = chain.forward_kinematics({'j1': 0.0})
    assert np.allclose(tf['A'], np.eye(4))
    assert np.allclose(tf['B'][2,3], 0.1, atol=1e-6)

