import numpy as np

from skeleton.base import BoneSpec
from soft.muscle_spec import MuscleSpec
from soft.muscle_agent import MuscleAgent
from sensors.receptor_spec import ReceptorSpec
from sensors.sensor_agent import SensorAgent
from neuro.neuro_agent import NeuroAgent
from neuro.pain_fatigue import PainFatigueModel
from damage.damage_engine import DamageEngine
from healing.healing_engine import HealingEngine
from autonomic.autonomic_agent import AutonomicAgent
from energy.energy_agent import EnergyAgent


def build_simple_muscle():
    spec = MuscleSpec("biceps", {}, {}, 300.0, 10.0, 10.0)
    muscle = MuscleAgent(spec, "elbow", 0.03)
    muscle.length_m = 0.1
    return muscle


def test_sensor_agent_firing():
    m = build_simple_muscle()
    rec = ReceptorSpec("msp", "muscle_spindle", 0.001, {"muscle": m.spec.name}, 10.0)
    sa = SensorAgent([rec], muscles=[m])
    sa.update(0.01)  # establish baseline
    m.length_m = 0.102
    sa.update(0.01)
    assert sa.firing["msp"] > 0.0


def test_stretch_reflex():
    m = build_simple_muscle()
    rec = ReceptorSpec("sp", "muscle_spindle", 0.0, {"muscle": m.spec.name}, 5.0)
    sa = SensorAgent([rec], muscles=[m])
    neuro = NeuroAgent(sa)
    sa.update(0.01)  # baseline
    m.length_m = 0.11
    sa.update(0.01)
    neuro.step(0.05)
    assert neuro.reflex[m.spec.name] > 0.0


def test_withdrawal_reflex():
    mflex = build_simple_muscle()
    reff = ReceptorSpec("noc", "nociceptor", 0.0, {"bone_uid": "B1"}, 1.0)
    bone = BoneSpec("B", "long", {}, [], {"length_cm": 5.0, "width_cm": 1.0, "thickness_cm": 1.0}, [], [], "", "", "B1")
    bone.damage = 1.0
    sa = SensorAgent([reff], muscles=[mflex], bones=[bone])
    neuro = NeuroAgent(sa, withdrawal_flexors=[mflex.spec.name])
    sa.update(0.01)
    neuro.step(0.05)
    assert neuro.reflex[mflex.spec.name] > 0.5


def test_pain_limits_force():
    model = PainFatigueModel()
    model.update_pain(8.0)
    scaled = model.apply(1.0)
    assert scaled < 0.25


def test_microfracture_heals():
    bone = BoneSpec("B", "long", {}, [], {"length_cm": 5.0, "width_cm": 1.0, "thickness_cm": 1.0}, [], [], "", "", "B1")
    engine = DamageEngine({bone.unique_id: bone})
    heal = HealingEngine(engine)
    engine.damage[bone.unique_id] = 1.0
    for _ in range(6 * 7):
        heal.update(24 * 3600)
    assert engine.damage[bone.unique_id] <= 0.2


def test_autonomic_temperature():
    auto = AutonomicAgent(EnergyAgent(), core_temp=37.5)
    auto.update(1.0)
    assert auto.sweat is True
