#!/usr/bin/env python
from __future__ import annotations

import argparse
import matplotlib.pyplot as plt

from skeleton.bones import load_bones
from healing.healing_engine import HealingEngine


def run(output="healing.png"):
    bones = load_bones("female_21_baseline")
    tib = next(b for b in bones if b.unique_id == "BONE_TIBIA_L")
    tib.material["E_base"] = 1.0
    engine = HealingEngine({tib.unique_id: tib})
    engine.start_healing(tib.unique_id)
    times = []
    strengths = []
    for _ in range(30):
        engine.update(7*24*3600/30)
        times.append(engine.time)
        strengths.append(tib.material.get("E",0.0))
    plt.plot(times, strengths)
    plt.xlabel("time (s)")
    plt.ylabel("E")
    plt.savefig(output)
    print(f"plot saved to {output}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--output", default="healing.png")
    args = ap.parse_args()
    run(args.output)
