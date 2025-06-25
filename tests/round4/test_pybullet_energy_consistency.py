import importlib.util
import pytest
from joints.joint_spec import hinge
from kinematics.kinematic_chain import KinematicChain
from physics.physics_agent import PhysicsAgent
from skeleton.datasets import bones_from_dataset


def test_pybullet_energy_consistency():
    if not importlib.util.find_spec("pybullet"):
        pytest.skip("pybullet not available")
    bones = bones_from_dataset('female_21_baseline')[:1]
    bones_dict = {b.unique_id: b for b in bones}
    chain = KinematicChain(bones[0], [], bones_dict)
    phys = PhysicsAgent(chain)
    phys.build()
    initial = phys.get_joint_state(bones[0].unique_id)[0][2]
    for _ in range(120):
        phys.step(1/60)
    after = phys.get_joint_state(bones[0].unique_id)[0][2]
    assert after <= initial
