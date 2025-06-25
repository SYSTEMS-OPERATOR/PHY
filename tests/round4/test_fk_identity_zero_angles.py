from joints.joint_spec import hinge
from kinematics.kinematic_chain import KinematicChain
from skeleton.datasets import bones_from_dataset


def test_fk_identity_zero_angles():
    bones = bones_from_dataset('female_21_baseline')[:2]
    bones_dict = {b.unique_id: b for b in bones}
    j = hinge('j', bones[0].unique_id, bones[1].unique_id, origin=(0,0,1))
    chain = KinematicChain(bones[0], [j], bones_dict)
    res = chain.forward_kinematics({'j':0.0})
    assert bones[1].unique_id in res
