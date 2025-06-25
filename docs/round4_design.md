# Round 4 Design Notes

This round introduces kinematics and physics features.  Bones are given
analytical geometry via `GeometryAgent` which computes volume, center of
mass and inertia. `JointSpec` and `KinematicChain` describe skeletal
connections and provide basic forward and inverse kinematics utilities.
`PhysicsAgent` uses PyBullet to simulate a chain in real time. URDF
export and a ROS2 TF broadcaster make it possible to view the skeleton
in common robotics tools.
