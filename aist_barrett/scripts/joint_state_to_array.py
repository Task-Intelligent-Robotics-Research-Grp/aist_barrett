#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node      import Node
from sensor_msgs.msg import JointState
from std_msgs.msg    import Float64MultiArray, MultiArrayDimension

######################################################################
#  class JointStateToArrayNode                                       #
######################################################################
class JointStateToArrayNode(Node):
    def __init__(self):
        super().__init__('joint_state_to_array')

        self._joints = self.declare_parameter('joints',
                                              ['bhand_left_finger_joint',
                                               'bhand_right_finger_joint',
                                               'bhand_middle_finger_joint',
                                               'bhand_spread_joint']).value
        self._sub = self.create_subscription(JointState, 'joint_states',
                                             self._joint_state_cb, 3)
        self._pub = self.create_publisher(Float64MultiArray, '~/out', 10)

    def _joint_state_cb(self, joint_state):
        data = Float64MultiArray()
        data.layout.dim.append(MultiArrayDimension(label='',
                                                   size=len(self._joints),
                                                   stride=1))
        data.layout.data_offset = 0
        for joint_name in self._joints:
            if joint_name in joint_state.name:
                data.data.append(
                    joint_state.position[joint_state.name.index(joint_name)])
            else:
                self.get_logger().error('unknown joint_name[%s]' % joint_name)
                return
        self._pub.publish(data)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateToArrayNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
