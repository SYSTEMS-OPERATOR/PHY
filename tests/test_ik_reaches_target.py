from geometry.geometry_agent import GeometryAgent
from skeleton.datasets import bones_from_dataset
from joints.joint_spec import JointSpec
from kinematics.kinematic_chain import KinematicChain
import numpy as np


def test_ik_reaches_target():
    bones = bones_from_dataset("female_21_baseline")[:2]
    for b in bones:
        b.set_material("organic")
        b.set_embodiment("physical", b.material)
        GeometryAgent(b).compute()
    origin = (0.0, 0.0, bones[0].dimensions.get("length_cm", 1) / 100.0)
    j = JointSpec.hinge(
        "j0",
        bones[0].unique_id,
        bones[1].unique_id,
        axis=(0, 0, 1),
        origin_xyz=origin,
    )
    chain = KinematicChain({b.unique_id: b for b in bones}, [j])
    target = (0.0, 0.0, bones[0].dimensions.get("length_cm", 1) / 100.0)
    angles = chain.inverse_kinematics(bones[1].unique_id, target)
    tf = chain.forward_kinematics(angles)
    pos = tf[bones[1].unique_id][:3, 3]
    assert np.linalg.norm(pos - np.array(target)) < 0.02
