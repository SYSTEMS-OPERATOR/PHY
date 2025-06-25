# Round 6 Neural and Healing Layer

This iteration adds a bare bones sensory and autonomic loop. Receptors attach to
muscles and bones and feed a small `NeuroAgent` which can trigger stretch and
withdrawal reflexes.  Damage accumulation is tracked on bones and a simple
`HealingEngine` reduces the damage over simulated weeks.  A `PainFatigueModel`
scales voluntary activation based on nociceptor firing.  The `AutonomicAgent`
monitors core temperature for shiver or sweat placeholders.

Two CLI demos illustrate the additions:

- `demo_reflexes.py` prints the reflex activation of a stretching biceps.
- `demo_healing.py` outputs a `healing_curve.png` plot of damage over six weeks.
