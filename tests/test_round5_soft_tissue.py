import numpy as np
import pytest
pytest.importorskip("pybullet")

from skeleton.bones import load_bones
from geometry.geometry_agent import GeometryAgent
from joints.joint_spec import hinge
from kinematics.kinematic_chain import KinematicChain
from physics.physics_agent import PhysicsAgent
from soft.muscle_spec import MuscleSpec
from soft.muscle_agent import MuscleAgent
from soft.ligament_agent import LigamentAgent
from control.control_agent import ControlAgent
from adaptation.wolff_engine import WolffAdaptationEngine
from energy.energy_agent import EnergyAgent


def build_elbow_system():
    bones = load_bones("female_21_baseline")
    hum = next(b for b in bones if b.unique_id == "BONE_HUMERUS_L")
    ulna = next(b for b in bones if b.unique_id == "BONE_ULNA_L")
    for b in (hum, ulna):
        b.set_material("organic")
        b.set_embodiment("physical", b.material)
        GeometryAgent(b).compute()
    j = hinge("elbow", hum.unique_id, ulna.unique_id, axis=(1, 0, 0), limit=(-180, 0), origin_xyz=(0, 0, hum.dimensions.get("length_cm", 30)/100))
    chain = KinematicChain({hum.unique_id: hum, ulna.unique_id: ulna}, [j], hum.unique_id)
    m_spec = MuscleSpec("biceps", {"bone_uid": hum.unique_id, "point": "distal"}, {"bone_uid": ulna.unique_id, "point": "proximal"}, 300.0, 10.0, 10.0)
    muscle = MuscleAgent(m_spec, "elbow", 0.03)
    ligament = LigamentAgent("elbow", 0.0, 10.0, 0.1)
    energy = EnergyAgent()
    agent = PhysicsAgent(chain, muscles=[muscle], ligaments=[ligament], controller=None, energy=energy)
    return agent, muscle, ligament


def test_muscle_agent_torque():
    spec = MuscleSpec("test", {}, {}, 100.0, 10.0, 10.0)
    m = MuscleAgent(spec, "j", 0.02)
    torque = m.update(0.01, 1.0)
    assert np.isclose(torque, 2.0)


def test_control_agent_filters_emg():
    ctrl = ControlAgent(["m"])
    act = ctrl.update(0.01, {"m": [0.5]})
    assert 0.0 <= act["m"] <= 0.5


def test_bicep_flexion():
    muscle = MuscleAgent(MuscleSpec("biceps", {}, {}, 300.0, 10.0, 10.0), "elbow", 0.03)
    angle = 0.0
    inertia = 10.0
    dt = 1 / 240
    for _ in range(240 * 2):
        torque = muscle.update(dt, 0.5)
        angle += (torque / inertia) * dt
    deg = np.degrees(angle)
    assert 40.0 <= deg <= 55.0


def test_ligament_resist_hyperextension():
    lig = LigamentAgent("elbow", 0.0, 10.0, 0.1)
    angle = 0.0
    vel = 0.0
    dt = 1 / 240
    inertia = 10.0
    for _ in range(240):
        torque = -20.0 + lig.update(np.degrees(angle), np.degrees(vel))
        acc = torque / inertia
        vel += acc * dt
        angle += vel * dt
    deg = np.degrees(angle)
    assert deg > -30.0


def test_wolff_adaptation_increases_density():
    bone = load_bones("female_21_baseline")[0]
    base = bone.material.get("density")
    engine = WolffAdaptationEngine({bone.unique_id: bone})
    for _ in range(10000):
        engine.record({bone.unique_id: base * 1.2})
    assert bone.material.get("density") > base * 1.05


def test_energy_accounting():
    energy = EnergyAgent()
    energy.accumulate(10.0, 1.0, 1.0)
    expected = 10.0 / 0.25 / 4184
    assert np.isclose(energy.daily_report(), expected)
