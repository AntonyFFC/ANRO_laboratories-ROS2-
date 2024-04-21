import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
# from dobot_msgs.action import PointToPoint
# from dobot_msgs.srv import GripperControl
from sensor_msgs.msg import JointState
import numpy as np
from geometry_msgs.msg import PointStamped
from math import isnan

class Reverse_kinematics(Node):

    def __init__(self):
        super().__init__('Reverse_Kinematics')
        self.subscription = self.create_subscription(PointStamped,'/clicked_point',self.clicked_point_callback,10)
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.l0 = 0.05
        self.l1 = 0.088
        self.l2 = 0.135
        self.l3 = 0.147
        self.l4 = 0.08

    def subscriber_callback(self, msg):
        self.angles = [msg.position[i] for i in range(len(msg.position))]
        self.calculate_angles()

    def clicked_point_callback(self, msg):
        # Extracting coordinates from the message
        if msg.point is not None:
            self.x = msg.point.x
            self.y = msg.point.y
            self.z = msg.point.z
            # Displaying the coordinates
            self.get_logger().info("Clicked Point Coordinates:")
            self.get_logger().info(f"X: {self.x}, Y: {self.y}, Z: {self.z}")
        else:
            self.get_logger().info("Received an empty point")

        

        self.calculate_angles()

    def calculate_angles(self):
        angle1 = 0.0
        angle2 = 0.0
        angle3 = 0.0
        angle4 = 0.0
        angle5 = 0.0
        pz = self.z + self.l4 - self.l0 - self.l1
        px = np.sqrt(self.x**2 + self.y**2) - 0.03
        alpha = np.arctan2(pz, px)
        m = np.sqrt(px**2 + pz**2)

        t3_num = m**2 - self.l2**2 - self.l3**2
        t3_den = 2*self.l2*self.l3


        angle3 = np.arccos(t3_num/t3_den) - np.pi/2 

        beta = np.arccos((self.l2**2 + m**2 - self.l3**2)/(2*self.l2*m))
        
        angle2 = np.pi/2 - (alpha + beta)

        angle4 = -(angle2 + angle3)

        # az = self.z+0.03-(self.l1+self.l0) # odjęcie wysokosci podstawyself
        # d = np.sqrt(np.square(self.x)+np.square(self.y)+np.square(az))
        # c = np.sqrt(np.square(self.x)+np.square(self.y))
        # h =np.sqrt(np.square(self.x)+np.square(az))
        # B1 = np.arctan(h/c)
        # B2 = np.arccos( (np.square(self.l3)-np.square(self.l2)-np.square(d)) / (-2*self.l2*d))
        # T = np.arccos( (np.square(d)-np.square(self.l2)-np.square(self.l3)) / (-2*self.l2*self.l3))
        # self.get_logger().info("Clicked Point variables:")
        # self.get_logger().info(f"az: {az}, d: {d}, c: {c}, h: {h}, B1: {B1}, B2: {B2}, T:{T}")
        angle1 = np.arctan(self.y/self.x)
        # angle2 = np.pi/2.0 - B1 -B2
        # angle3 = np.pi/2.0 - T
        # angle4 = -angle2-angle3
        angle5 = 0.0

        if angle1 < -1.548 or angle1 > 1.548 or angle2 < 0 or angle2 > 1.484 or angle3 < -0.175 or angle3 > 1.571:
            angle1 = 0.0
            angle2 = 0.0
            angle3 = 0.0
            angle4 = 0.0
            angle5 = 0.0

        self.get_logger().info("Clicked Point Angles:")
        self.get_logger().info(f"a1: {angle1}, a2: {angle2}, a3: {angle3}")
        self.publish_joint_states([angle1, angle2, angle3, angle4, angle5])

    def publish_joint_states(self, answer_vector):
        self.get_logger().info("Answer Vector: {}".format(answer_vector))
        self.get_logger().info("Type of answer_vector: {}".format(type(answer_vector)))
        self.get_logger().info("Length of answer_vector: {}".format(len(answer_vector)))
        answer_vector = [0.0 if isnan(val) else val for val in answer_vector]

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['rotating_base_joint', 'arm_rotbase_joint', 'arm_arm_joint', 'effector_lean_joint', 'effector_rot_joint']
        joint_state.position = answer_vector
        self.publisher.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    client = Reverse_kinematics()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
