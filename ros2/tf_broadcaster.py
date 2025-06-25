from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from physics.physics_agent import PhysicsAgent


class TFBroadcaster(Node):
    def __init__(self, phys: PhysicsAgent) -> None:
        super().__init__('tf_broadcaster')
        self.phys = phys
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.tick)

    def tick(self) -> None:
        for uid in self.phys.bodies:
            pos, orn = self.phys.get_joint_state(uid)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = uid
            t.transform.translation.x = pos[0]
            t.transform.translation.y = pos[1]
            t.transform.translation.z = pos[2]
            t.transform.rotation.x = orn[0]
            t.transform.rotation.y = orn[1]
            t.transform.rotation.z = orn[2]
            t.transform.rotation.w = orn[3]
            self.br.sendTransform(t)


def main():
    rclpy.init()
    # stub chain
    node = TFBroadcaster(None)  # type: ignore
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
