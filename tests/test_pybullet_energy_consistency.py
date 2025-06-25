import numpy as np
from geometry.geometry_agent import GeometryAgent
from skeleton.datasets import bones_from_dataset
from joints.joint_spec import JointSpec
from kinematics.kinematic_chain import KinematicChain
from physics.physics_agent import PhysicsAgent


def test_pybullet_energy_consistency():
    bones = bones_from_dataset("female_21_baseline")[:1]
    bone = bones[0]
    bone.set_material("organic")
    bone.set_embodiment("physical", bone.material)
    GeometryAgent(bone).compute()
    chain = KinematicChain({bone.unique_id: bone}, [])
    physics = PhysicsAgent(chain)
    mass = bone.mass_kg() or 1.0
    start_height = 1.0
    g = 9.81
    potential_start = mass * g * start_height
    steps = int(5 / 0.01)
    for _ in range(steps):
        physics.step(0.01)
    pos, _ = physics.get_joint_state(bone.unique_id)
    potential_end = mass * g * pos[2]
    assert potential_end < potential_start
