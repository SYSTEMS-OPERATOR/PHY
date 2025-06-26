# Round 7 Cognition Overview

This round introduces perception modules, a simple cortical policy and
a forward model placeholder.  Environments are lightweight Gym tasks
for CI and unit tests.  A curriculum runner chains tasks of increasing
complexity.  Emotion state scales the learning rate.

The PPO configuration can be customised via YAML files passed to the
`CorticalAgent`.
