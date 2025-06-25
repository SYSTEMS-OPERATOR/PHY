from __future__ import annotations

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped
    from tf2_ros.transform_broadcaster import TransformBroadcaster
except Exception:  # pragma: no cover - ROS not available
    rclpy = None

from physics.physics_agent import PhysicsAgent


class TFBroadcaster(Node):
    def __init__(self, physics: PhysicsAgent) -> None:
        super().__init__("tf_broadcaster")
        self.physics = physics
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.broadcast)

    def broadcast(self) -> None:
        for uid, body in self.physics.bodies.items():
            pos, quat = self.physics.get_joint_state(uid)
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            msg.child_frame_id = uid
            msg.transform.translation.x = pos[0]
            msg.transform.translation.y = pos[1]
            msg.transform.translation.z = pos[2]
            msg.transform.rotation.x = quat[0]
            msg.transform.rotation.y = quat[1]
            msg.transform.rotation.z = quat[2]
            msg.transform.rotation.w = quat[3]
            self.br.sendTransform(msg)


def start_broadcaster(physics: PhysicsAgent) -> None:
    if rclpy is None:
        raise RuntimeError("rclpy is not available")
    rclpy.init()
    node = TFBroadcaster(physics)
    rclpy.spin(node)
    rclpy.shutdown()
