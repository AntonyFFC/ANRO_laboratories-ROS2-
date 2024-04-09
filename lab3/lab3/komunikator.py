import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import GripperControl
from sensor_msgs.msg import JointState
import numpy as np


class Komunikator(Node):

    def __init__(self):
        super().__init__('komunikator')
        self.subscriber = self.create_subscription(JointState, 'dobot_joint_states', self.subscriber_callback, 10)
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.angles = []

    def subscriber_callback(self, msg):
        self.angles = [msg.position[i] for i in range(len(msg.position))]
        self.calculate_angles()

    def calculate_angles(self):
        angle1 = self.angles[0]
        angle2 = -self.angles[1]
        angle3 = -self.angles[2]
        angle5 = -self.angles[3]
        angle3 = angle3 - angle2 
        angle4 = -angle2 - angle3
        self.publish_joint_states([angle1, -angle2, -angle3, -angle4, -angle5])

    def publish_joint_states(self, answer_vector):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['rotating_base_joint', 'arm_rotbase_joint', 'arm_arm_joint', 'effector_lean_joint', 'effector_rot_joint']
        joint_state.position = answer_vector
        self.publisher.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    client = Komunikator()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
