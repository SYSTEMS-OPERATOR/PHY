# Round 4 Design

This round introduces dynamic simulation features. New modules provide basic geometry generation, kinematic modeling and a minimal PyBullet interface. A small ROS2 broadcaster is stubbed for future use.

* `GeometryAgent` attaches simple mesh data, COM and inertia to each `BoneSpec`.
* `JointSpec` describes joints and is used by the new `KinematicChain` class for forward and inverse kinematics.
* `PhysicsAgent` wraps PyBullet and builds a toy multibody from a chain.
* URDF files can be exported via `export_urdf`.
* A CLI entry point `run_dynamics.py` demonstrates basic stepping.
* Tests cover inertia positivity, forward/inverse kinematics and a short PyBullet drop check.
