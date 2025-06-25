import numpy as np

from skeleton.bones import load_bones
from geometry.geometry_agent import GeometryAgent
from joints.joint_spec import hinge
from kinematics.kinematic_chain import KinematicChain
from physics.physics_agent import PhysicsAgent
from soft.muscle_spec import MuscleSpec
from soft.muscle_agent import MuscleAgent
from control.control_agent import ControlAgent
from sensors.receptor_spec import ReceptorSpec
from sensors.sensor_agent import SensorAgent
from neuro.neuro_agent import NeuroAgent
from neuro.pain_fatigue import PainFatigueModel
from damage.damage_engine import DamageEngine
from healing.healing_engine import HealingEngine


def build_simple_elbow():
    bones = load_bones("female_21_baseline")
    hum = next(b for b in bones if b.unique_id == "BONE_HUMERUS_L")
    ulna = next(b for b in bones if b.unique_id == "BONE_ULNA_L")
    for b in (hum, ulna):
        b.set_material("organic")
        b.set_embodiment("physical", b.material)
        GeometryAgent(b).compute()
    j = hinge("elbow", hum.unique_id, ulna.unique_id, axis=(1,0,0), limit=(-180,0), origin_xyz=(0,0,hum.dimensions.get("length_cm",30)/100))
    chain = KinematicChain({hum.unique_id: hum, ulna.unique_id: ulna}, [j], hum.unique_id)
    muscle = MuscleAgent(MuscleSpec("biceps", {"bone_uid": hum.unique_id}, {"bone_uid": ulna.unique_id}, 300.0, 10.0, 10.0), "elbow", 0.03)
    agent = PhysicsAgent(chain, muscles=[muscle], controller=None)
    return agent, muscle


def test_sensor_firing():
    agent, muscle = build_simple_elbow()
    rec = ReceptorSpec("s", "muscle_spindle", 0.1, {"bone_uid": muscle.spec.name})
    sensor = SensorAgent(rec, muscle=muscle, physics=agent, joint_name="elbow")
    agent.step(1/240)
    f1 = sensor.update(1/240)
    agent.step(1/240)
    f2 = sensor.update(1/240)
    assert f2 >= 0.0


def test_stretch_reflex():
    agent, muscle = build_simple_elbow()
    spindle = SensorAgent(ReceptorSpec("sp","muscle_spindle",0.0,{"bone_uid":""},1.0), muscle=muscle, physics=agent, joint_name="elbow")
    neuro = NeuroAgent([spindle])
    agent.neuro = neuro
    ctrl = ControlAgent([muscle.spec.name])
    agent.controller = ctrl
    spindle.firing_hz = 5.0
    neuro.stretch_reflex()
    torque = muscle.update(1/240, neuro.reflex_commands.get(muscle.spec.name, 0.0))
    assert torque > 0.0


def test_withdrawal_reflex():
    agent, muscle = build_simple_elbow()
    noc = SensorAgent(ReceptorSpec("n","nociceptor",0.0,{"bone_uid":""},1.0), muscle=muscle)
    neuro = NeuroAgent([noc])
    agent.neuro = neuro
    muscle.damage_ratio = 1.0
    reflex = neuro.step(1/240)
    assert reflex[muscle.spec.name] > 0.0


def test_pain_limits_force():
    model = PainFatigueModel(pain=8.0, fatigue=0.0)
    eff = model.activation_efficiency(1.0)
    assert eff < 0.7


def test_damage_event_and_heal():
    agent, muscle = build_simple_elbow()
    dmg = DamageEngine({b.unique_id: b for b in agent.chain.bones.values()})
    heal = HealingEngine({b.unique_id: b for b in agent.chain.bones.values()})
    agent.damage = dmg
    agent.healing = heal
    loads = {muscle.spec.name: 2e6}
    dmg.accumulate({"BONE_ULNA_L":2e6}, 0.01)
    heal.start_healing("BONE_ULNA_L")
    for _ in range(10):
        heal.update(6*7*24*3600/10)
    bone = agent.chain.bones["BONE_ULNA_L"]
    assert bone.material.get("E",0.0) >= bone.material.get("E_base",1.0)*0.8
