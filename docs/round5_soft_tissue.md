# Round 5 Soft Tissue Overview

This iteration introduces simple soft–tissue structures and a bioelectric control
layer on top of the digital skeleton.  Muscles and ligaments are represented
with light–weight data classes and agents.  A control pipeline converts EMG
samples into activations that drive the new `MuscleAgent` objects.  Bone density
can now adapt under repeated load via a very small Wolff–law inspired engine and
energy consumption is estimated from joint work.

The implementation is intentionally minimal.  Each muscle acts on a specific
joint with a constant moment arm and produces torque proportional to its current
activation.  Ligaments resist motion away from a rest angle.  The physics agent
collects all torques each step and optionally tracks energy and adaptation.

Two CLI demos show the features in action:

- `run_emg_demo.py` flexes the elbow with a 50 % EMG signal.
- `run_adaptation_demo.py` applies repetitive knee loading to trigger density
  changes.
