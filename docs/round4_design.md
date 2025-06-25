# Round 4 Design

This iteration introduces a minimal mechanics stack for the skeleton dataset.
It adds geometry and kinematics utilities as well as a lightweight physics
wrapper around PyBullet.  The focus is correctness rather than fidelity.

## Components

- **GeometryAgent** computes a primitive shape for a `BoneSpec`. Long bones are
  treated as cylinders, while flat and irregular bones fall back to a box
  approximation. Irregular shapes carry a `mass_distribution` flag to mark
  non-uniform mass. The agent stores the volume, centre of mass and inertia
  tensor on the bone.
- **JointSpec** describes the connection between bones. Factory helpers create
  common hinge and pivot joints.
- **KinematicChain** builds a tree from bones and joints. It supports forward and
  inverse kinematics using a damped least squares method.
- **PhysicsAgent** uses PyBullet to simulate the chain.  The agent loads a URDF
  generated on the fly and exposes simple methods for stepping the simulation and
  querying joints.
- **URDF exporter** serialises the chain to a minimal URDF file.
- **TF broadcaster** publishes PyBullet poses as ROS2 TF frames.

A simple CLI entry point `run_dynamics.py` demonstrates a standing and drop
configuration.

