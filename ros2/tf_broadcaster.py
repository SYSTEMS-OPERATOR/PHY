from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from physics.physics_agent import PhysicsAgent


class TFBroadcaster(Node):
    def __init__(self, agent: PhysicsAgent):
        super().__init__('tf_broadcaster')
        self.agent = agent
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.broadcast)

    def broadcast(self) -> None:
        for uid in self.agent.joint_map:
            state = self.agent.get_joint_state(uid)
            if state is None:
                continue
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = uid
            pos = state[0]
            t.transform.translation.x = pos
            self.br.sendTransform(t)


def run(agent: PhysicsAgent) -> None:
    rclpy.init()
    node = TFBroadcaster(agent)
    rclpy.spin(node)
    rclpy.shutdown()

