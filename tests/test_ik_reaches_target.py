import numpy as np
from skeleton.base import BoneSpec
from joints.joint_spec import hinge
from kinematics.kinematic_chain import KinematicChain


def test_ik_reaches_target():
    b1 = BoneSpec('b1','long',{},[],{'length_cm':10,'width_cm':1,'thickness_cm':1},[],[],"","",'B1')
    b2 = BoneSpec('b2','long',{},[],{'length_cm':10,'width_cm':1,'thickness_cm':1},[],[],"","",'B2')
    b3 = BoneSpec('b3','long',{},[],{'length_cm':10,'width_cm':1,'thickness_cm':1},[],[],"","",'B3')
    bones = {b1.unique_id: b1, b2.unique_id: b2, b3.unique_id: b3}
    joints = [
        hinge('j1', b1.unique_id, b2.unique_id, axis=(0,1,0), origin=(0,0,0)),
        hinge('j2', b2.unique_id, b3.unique_id, axis=(0,1,0), origin=(0,0,0)),
    ]
    chain = KinematicChain(bones, joints, b1.unique_id)
    target = np.array([20.0, 0.0, 0.0])
    angles = chain.inverse_kinematics(target, b3.unique_id)
    tfs = chain.forward_kinematics(angles)
    pos = tfs[b3.unique_id][:3,3]
    assert np.linalg.norm(pos - target) < 0.02
