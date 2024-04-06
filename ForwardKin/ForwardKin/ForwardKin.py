import rclpy
from rclpy.node import Node
import numpy as np
from numpy import sin, cos 
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState


class ForwardKin(Node):

    def __init__(self):
        super().__init__("ForwardKin")
        self.pose_publ = self.create_publisher(PoseStamped, 'end_pose', 10)
        self.joint_subsc = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.angles = []

    def calculate_position(self):
        matrixes = []
        # T01
        matrixes.append(np.matrix([
                    [cos(self.angles[0]), -sin(self.angles[0]), 0, 0],
                    [sin(self.angles[0]), cos(self.angles[0]), 0, 0],
                    [0, 0, 1, 0.05], # this is the length l0
                    [0, 0, 0, 1]                                   
        ]))
        # T12
        matrixes.append(np.matrix([
                    [cos(self.angles[1]), 0, sin(self.angles[1]), 0],
                    [0, 1, 0, 0],
                    [-sin(self.angles[1]), 0, cos(self.angles[1]), 0.088], # this is the length l1
                    [0, 0, 0, 1]
        ]))
        # T23
        matrixes.append(np.matrix([
                    [cos(self.angles[2]), 0, sin(self.angles[2]), 0],
                    [0, 1, 0, 0],
                    [-sin(self.angles[2]), 0, cos(self.angles[2]), 0.135],# this is the length l2
                    [0, 0, 0, 1]
        ]))
        # T34
        matrixes.append(np.matrix([
                    [cos(self.angles[3]), 0, sin(self.angles[3]), 0.147],# this is the length l3
                    [0, 1, 0, 0],
                    [-sin(self.angles[3]), 0, cos(self.angles[3]), 0],
                    [0, 0, 0, 1]
        ]))
        # T45
        matrixes.append(np.matrix([
                    [cos(self.angles[4]), -sin(self.angles[4]), 0, 0.03], # this is the length of l4/2 - 0.02 + 0.03
                    [sin(self.angles[4]), cos(self.angles[4]), 0, 0],
                    [0, 0, 1, -0.03],
                    [0, 0, 0, 1]                                   
        ]))
        # T56
        matrixes.append(np.matrix([
                    [1, 0, 0, 0],
                    [0, cos(np.pi), -sin(np.pi), 0],
                    [0, sin(np.pi), cos(np.pi), 0],
                    [0, 0, 0, 1]                                   
        ]))

        for i in range(len(self.angles)):
            matrixes[i+1] = np.matmul(matrixes[i], matrixes[i+1])

        return matrixes[5]
        
    def joint_state_callback(self, msg):
        self.angles = msg.position
        position_matrix = self.calculate_position()
        self.publish_position(position_matrix)
    
    def publish_position(self, position_matrix):
        position = position_matrix[:3, 3]
        orientation = position_matrix[:3, :3]
        quaternion = R.from_matrix(orientation).as_quat()

        pose = Pose()
        pose.position.x = position.item(0,0)
        pose.position.y = position.item(1,0)
        pose.position.z = position.item(2,0)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        pose_stamped_msg.header.frame_id = 'base'
        pose_stamped_msg.pose = pose
        self.pose_publ.publish(pose_stamped_msg)


def main(args=None):
    rclpy.init(args=args)
    client = ForwardKin()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()
        

if __name__ == "__main__":
    main()
