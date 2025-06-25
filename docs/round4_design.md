# Round 4 Design Notes

This round introduces mechanical layers for the armature. Bones now carry
simplified geometry and inertial data produced by `GeometryAgent`.  A light
`JointSpec` model describes the connections between bones and the
`KinematicChain` utility performs forward kinematics.  A thin
`PhysicsAgent` uses PyBullet to build a basic rigid body representation.
URDF export is provided for integration with external tools.  A ROS2 TF
broadcaster is stubbed for real-time visualisation.
