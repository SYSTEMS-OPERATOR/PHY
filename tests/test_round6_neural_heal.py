from soft.muscle_spec import MuscleSpec
from soft.muscle_agent import MuscleAgent
from sensors.receptor_spec import ReceptorSpec
from sensors.sensor_agent import SensorAgent
from neuro.neuro_agent import NeuroAgent
from neuro.pain_fatigue import PainFatigueModel
from healing.healing_engine import HealingEngine
from skeleton.bones import BoneSpec


def test_stretch_reflex():
    m = MuscleAgent(MuscleSpec("biceps", {}, {}, 300.0, 10.0, 10.0), "elbow", 0.03)
    sensor = SensorAgent(ReceptorSpec("biceps", "muscle_spindle", -0.1, {}), muscle=m)
    neuro = NeuroAgent({m.spec.name: m}, [sensor])
    m.length_m = 0.2
    neuro.step(0.05)
    assert m.spec.activation > 0.0


def test_withdrawal_reflex():
    flexor = MuscleAgent(MuscleSpec("hip_flexor", {}, {}, 300.0, 10.0, 10.0), "hip", 0.03)
    ext = MuscleAgent(MuscleSpec("hip_extensor", {}, {}, 300.0, 10.0, 10.0), "hip", 0.03)
    nocicept = SensorAgent(ReceptorSpec("pain", "nociceptor", -1.0, {}), muscle=flexor)
    neuro = NeuroAgent({flexor.spec.name: flexor, ext.spec.name: ext}, [nocicept])
    flexor.spec.activation = 0.0
    ext.spec.activation = 1.0
    flexor.spec.activation = 0.0
    nocicept.update(0.0)
    neuro.step(0.01)
    assert flexor.spec.activation == 1.0
    assert ext.spec.activation == 0.0


def test_microfracture_heals():
    bone = BoneSpec("tibia", "long", {}, [], {"length_cm":1,"width_cm":1,"thickness_cm":1}, [], [], "", "", "T1")
    bone.material["Youngs_modulus_GPa"] = 1.0
    heal = HealingEngine(bone)
    heal.start_healing(0.0)
    heal.update(42.0)
    assert bone.material["Youngs_modulus_GPa"] >= 0.8


def test_pain_limits_force():
    model = PainFatigueModel(pain_level=8.0)
    eff = model.activation_efficiency()
    assert eff <= 0.6
