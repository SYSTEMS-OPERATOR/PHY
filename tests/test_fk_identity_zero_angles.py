import numpy as np
from skeleton.datasets import bones_from_dataset
from joints.joint_spec import hinge
from kinematics.kinematic_chain import KinematicChain


def test_fk_identity_zero_angles():
    bones = {b.unique_id: b for b in bones_from_dataset('female_21_baseline')[:2]}
    joints = [hinge('j1', list(bones)[0], list(bones)[1])]
    chain = KinematicChain(bones, joints, list(bones)[0])
    tfs = chain.forward_kinematics({'j1': 0.0})
    expected = np.eye(4)
    length = bones[list(bones)[1]].dimensions.get('length_cm')
    expected[0,3] = length
    assert np.allclose(tfs[list(bones)[1]], expected)
