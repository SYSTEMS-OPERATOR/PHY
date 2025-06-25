# Round 6 Neural and Healing Overview

This iteration adds a very small reflex and healing layer on top of the physics
stack.  The goal is demonstration and test coverage rather than fidelity.

## Components

- **SensorAgent** wraps a simple `ReceptorSpec` and outputs a firing rate based
  on muscle length or tension.
- **NeuroAgent** collects these sensor readings and applies primitive stretch and
  withdrawal reflexes by directly modifying muscle activations.
- **DamageEngine** records loads that exceed yield or ultimate thresholds.
- **HealingEngine** restores lost modulus over several simulated weeks.
- **AutonomicAgent** slightly adjusts energy expenditure when core temperature
  drifts.

The full integration sequence updates reflexes before voluntary control,
processes muscles and ligaments, records damage and healing, steps the physics
engine and finally refreshes sensors.
