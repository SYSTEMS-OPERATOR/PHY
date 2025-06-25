#!/usr/bin/env python
from __future__ import annotations

import matplotlib.pyplot as plt

from skeleton.base import BoneSpec
from damage.damage_engine import DamageEngine
from healing.healing_engine import HealingEngine


def run_demo() -> None:
    bone = BoneSpec("B", "long", {}, [], {"length_cm": 5.0, "width_cm": 1.0, "thickness_cm": 1.0}, [], [], "", "", "B1")
    dmg = DamageEngine({bone.unique_id: bone})
    heal = HealingEngine(dmg)
    dmg.damage[bone.unique_id] = 1.0
    weeks = []
    frac = []
    for w in range(6):
        heal.update(7 * 24 * 3600)
        weeks.append(w + 1)
        frac.append(dmg.damage[bone.unique_id])
    plt.plot(weeks, frac)
    plt.xlabel("weeks")
    plt.ylabel("damage")
    plt.savefig("healing_curve.png")


if __name__ == "__main__":
    run_demo()
