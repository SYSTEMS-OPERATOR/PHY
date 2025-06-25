#!/usr/bin/env python
from __future__ import annotations

from sensors.receptor_spec import ReceptorSpec
from sensors.sensor_agent import SensorAgent
from neuro.neuro_agent import NeuroAgent
from soft.muscle_spec import MuscleSpec
from soft.muscle_agent import MuscleAgent


def main() -> None:
    muscle = MuscleAgent(MuscleSpec("biceps_flexor", {}, {}, 300.0, 10.0, 10.0), "elbow", 0.03)
    sensor = SensorAgent(ReceptorSpec("biceps_flexor", "muscle_spindle", 0.0, {}), muscle=muscle)
    neuro = NeuroAgent({muscle.spec.name: muscle}, [sensor])
    for _ in range(10):
        neuro.step(0.01)
        print(f"activation: {muscle.spec.activation:.2f}")


if __name__ == "__main__":
    main()
