#!/usr/bin/env python
from __future__ import annotations

from skeleton.bones import load_bones
from healing.healing_engine import HealingEngine


def main() -> None:
    bone = load_bones("female_21_baseline")[0]
    engine = HealingEngine(bone)
    engine.start_healing(0.0)
    for day in range(0, 50, 5):
        engine.update(day)
        print(day, bone.material.get("Youngs_modulus_GPa"))


if __name__ == "__main__":
    main()
