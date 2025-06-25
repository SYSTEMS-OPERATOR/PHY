from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import pybullet as pb

from physics.physics_agent import PhysicsAgent


class TFBroadcaster(Node):
    def __init__(self, agent: PhysicsAgent, base_frame: str = 'world') -> None:
        super().__init__('tf_broadcaster')
        self.agent = agent
        self.base_frame = base_frame
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.publish)

    def publish(self) -> None:
        for uid in self.agent.chain.bones:
            if uid == self.agent.chain.root_uid:
                trans, orn = self.agent.chain.bones[uid].position, (0,0,0,1)
            else:
                idx = self.agent.joint_map.get(uid)
                if idx is None:
                    continue
                state = pb.getLinkState(self.agent.robot, idx, physicsClientId=self.agent.client)
                trans, orn = state[0], state[1]
            msg = TransformStamped()
            msg.header.frame_id = self.base_frame
            msg.child_frame_id = uid
            msg.transform.translation.x = trans[0]
            msg.transform.translation.y = trans[1]
            msg.transform.translation.z = trans[2]
            msg.transform.rotation.x = orn[0]
            msg.transform.rotation.y = orn[1]
            msg.transform.rotation.z = orn[2]
            msg.transform.rotation.w = orn[3]
            self.br.sendTransform(msg)

