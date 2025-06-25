from geometry.geometry_agent import GeometryAgent
from skeleton.datasets import bones_from_dataset
from joints.joint_spec import JointSpec
from kinematics.kinematic_chain import KinematicChain
import numpy as np


def test_fk_identity_zero_angles():
    bones = bones_from_dataset("female_21_baseline")[:2]
    for b in bones:
        b.set_material("organic")
        b.set_embodiment("physical", b.material)
        GeometryAgent(b).compute()
    j = JointSpec.hinge("j0", bones[0].unique_id, bones[1].unique_id)
    chain = KinematicChain({b.unique_id: b for b in bones}, [j])
    tf = chain.forward_kinematics({"j0": 0.0})
    np.testing.assert_allclose(tf[bones[0].unique_id], np.eye(4))
    np.testing.assert_allclose(tf[bones[1].unique_id], np.eye(4))
