# Round 6 Neural and Healing Overview

This round introduces a minimal neuro-sensory layer, damage detection and a bone
healing placeholder. Sensors can be attached to muscles or joints to detect
stretch, tension and pain. The `NeuroAgent` aggregates these signals and
produces simple reflex activations that add to the voluntary control stream.

A tiny `DamageEngine` flags bones that exceed material limits and a
`HealingEngine` gradually restores strength over six simulated weeks.  Pain and
fatigue reduce voluntary activation via the `PainFatigueModel`.  An
`AutonomicAgent` adjusts basal energy based on core temperature.

The implementation is intentionally simple yet sufficient for lightweight unit
tests and CI demos.
