#!/usr/bin/env python
from __future__ import annotations

import argparse
import time

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
from energy.energy_agent import EnergyAgent


def build():
    bones = load_bones("female_21_baseline")
    hum = next(b for b in bones if b.unique_id == "BONE_HUMERUS_L")
    ulna = next(b for b in bones if b.unique_id == "BONE_ULNA_L")
    for b in (hum, ulna):
        b.set_material("organic")
        b.set_embodiment("physical", b.material)
        GeometryAgent(b).compute()
    j = hinge("elbow", hum.unique_id, ulna.unique_id, axis=(1,0,0), limit=(-180,0), origin_xyz=(0,0,hum.dimensions.get("length_cm",30)/100))
    chain = KinematicChain({hum.unique_id: hum, ulna.unique_id: ulna}, [j], hum.unique_id)
    muscle = MuscleAgent(MuscleSpec("biceps", {"bone_uid": hum.unique_id}, {"bone_uid": ulna.unique_id},300.0,10.0,10.0),"elbow",0.03)
    ctrl = ControlAgent([muscle.spec.name])
    sensor = SensorAgent(ReceptorSpec("sp","muscle_spindle",0.0,{"bone_uid":""}), muscle=muscle, physics=None, joint_name="elbow")
    agent = PhysicsAgent(chain, muscles=[muscle], controller=ctrl, energy=EnergyAgent(), neuro=NeuroAgent([sensor]))
    sensor.physics = agent
    return agent, ctrl


def run(duration=1.0, output=None):
    agent, ctrl = build()
    t = 0.0
    while t < duration:
        ctrl.update(1/240, {"biceps": [0.0]})
        agent.step(1/240)
        t += 1/240
    angle = agent.get_joint_state("elbow")
    print(f"final angle: {angle:.2f} deg")
    if output:
        with open(output, "w") as fh:
            fh.write("demo complete")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", default=None)
    args = ap.parse_args()
    run(output=args.output)
