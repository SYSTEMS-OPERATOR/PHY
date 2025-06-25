#!/usr/bin/env python
from __future__ import annotations

from sensors.receptor_spec import ReceptorSpec
from sensors.sensor_agent import SensorAgent
from neuro.neuro_agent import NeuroAgent
from soft.muscle_spec import MuscleSpec
from soft.muscle_agent import MuscleAgent


def run_demo() -> None:
    muscle = MuscleAgent(MuscleSpec("biceps", {}, {}, 300.0, 10.0, 10.0), "elbow", 0.03)
    sensor = SensorAgent([ReceptorSpec("sp", "muscle_spindle", 0.0, {"muscle": "biceps"}, 5.0)], muscles=[muscle])
    neuro = NeuroAgent(sensor)
    for i in range(10):
        muscle.length_m = 0.1 + i * 0.003
        sensor.update(0.01)
        neuro.step(0.05)
        print(f"step {i} reflex {neuro.reflex.get('biceps', 0.0):.2f}")


if __name__ == "__main__":
    run_demo()
