import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Point, Quaternion
from sensor_msgs.msg import JointState
import numpy as np
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from transforms3d.euler import euler2quat

from numpy import sin, cos
import random

camera_effector_difference_x = 0.1
camera_effector_difference_z = 0

class MarkerSender(Node):
    def __init__(self):
        super().__init__('marker_sender')
        self.cx = 0
        self.cy = 0
        self.cz = 0.011
        self.cangle = 0

        self.px = 0
        self.py = 0
        self.pz = 0.0005
        self.pangle = 0

        self.generate_markers_position()
        self.finish_subscriber = self.create_subscription(Bool, 'finished', self.finished_callback, 10)
        self.finish_publisher = self.create_publisher(Bool, 'finished', 10)
        self.joint_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        self.marker_publisher = self.create_publisher(PoseArray, 'camera_link', 10)


    def generate_markers_position(self):
        self.cx = 0.18 + random.randint(-5, 5)*0.01
        self.cy = random.randint(-10, 10)*0.01
        self.cangle = random.randint(-4, 4)*np.pi/10

        self.px = 0.18 + random.randint(-5, 5)*0.01
        self.py = random.randint(-3, 3)*0.01
        self.pangle = random.randint(-4, 4)*np.pi/10

    def finished_callback(self, msg: Bool):
        if msg.data is True:
            self.generate_markers_position()
            new_msg = Bool()
            new_msg.data = False
            self.finish_publisher.publish(new_msg)


    def joint_states_callback(self, msg: JointState):
        theta_vector = msg.position
        joint_count = len(theta_vector)

        matrixes = []

        matrixes.append(np.matrix([[cos(theta_vector[0]), -sin(theta_vector[0]), 0, 0],
                                   [sin(theta_vector[0]), cos(theta_vector[0]), 0, 0],
                                   [0, 0, 1, 0.05],
                                   [0, 0, 0, 1]                                   
                                   ]))
        
        matrixes.append(np.matrix([
                    [cos(theta_vector[1]), 0, sin(theta_vector[1]), 0],
                    [0, 1, 0, 0],
                    [-sin(theta_vector[1]), 0, cos(theta_vector[1]), 0.088],
                    [0, 0, 0, 1]
        ]))

        matrixes.append(np.matrix([
                    [cos(theta_vector[2]), 0, sin(theta_vector[2]), 0],
                    [0, 1, 0, 0],
                    [-sin(theta_vector[2]), 0, cos(theta_vector[2]), 0.135],
                    [0, 0, 0, 1]
        ]))

        matrixes.append(np.matrix([
                    [cos(theta_vector[3]), 0, sin(theta_vector[3]), 0.147],
                    [0, 1, 0, 0],
                    [-sin(theta_vector[3]), 0, cos(theta_vector[3]), 0],
                    [0, 0, 0, 1]
        ]))

        matrixes.append(np.matrix([[cos(theta_vector[4]), -sin(theta_vector[4]), 0, 0.03],
                                   [sin(theta_vector[4]), cos(theta_vector[4]), 0, 0],
                                   [0, 0, 1, -0.024-0.04-0.005],
                                   [0, 0, 0, 1]                                   
                                   ]))

        matrixes.append(np.matrix([
                    [1, 0, 0, 0],
                    [0, cos(np.pi), -sin(np.pi), 0],
                    [0, sin(np.pi), cos(np.pi), 0],
                    [0, 0, 0, 1]
        ]))
        

        for i in range(joint_count):
            R = np.matmul(matrixes[i], matrixes[i+1])
            matrixes[i+1] = R

        self.tab = matrixes[5]

        self.tbc = np.matrix([
            [1, 0, 0, camera_effector_difference_x],
            [0, 1, 0, 0],
            [0, 0, 1, -camera_effector_difference_z],
            [0, 0, 0, 1]
        ])
        self.tac = np.matmul(self.tab, self.tbc)
        self.publish_markers()

    def publish_markers(self):
        cube_pose = np.array([self.cx, self.cy, self.cz, 1])
        paper_pose = np.array([self.px, self.py, self.pz, 1])
        cp = np.matmul(np.linalg.inv(self.tac), cube_pose.T)
        pp = np.matmul(np.linalg.inv(self.tac), paper_pose.T)

        cube_point = Point(x=cp.item(0,0), y=cp.item(0,1), z=cp.item(0,2))
        paper_point = Point(x=pp.item(0,0), y=pp.item(0,1), z=pp.item(0,2))

        cube_roll = 0.0
        cube_pitch = 0.0
        cube_yaw = self.cangle

        self.get_logger().info(f"cube angle: {cube_yaw}")

        cube_quat = euler2quat(cube_roll, cube_pitch, cube_yaw)

        paper_roll = 0.0
        paper_pitch = 0.0
        paper_yaw = self.pangle

        paper_quat = euler2quat(paper_roll, paper_pitch, paper_yaw)
        

        self.cube_marker_pose = Pose(position=cube_point,
                                          orientation=Quaternion(x=cube_quat[1], y=cube_quat[2], z=cube_quat[3], w=cube_quat[0]))
        
        self.paper_marker_pose = Pose(position=paper_point,
                                          orientation=Quaternion(x=paper_quat[1], y=paper_quat[2], z=paper_quat[3], w=paper_quat[0]))

        marker_pose_array = PoseArray()
        marker_pose_array.header.stamp = self.get_clock().now().to_msg()
        marker_pose_array.header.frame_id = 'camera_link'
        marker_pose_array.poses = [self.cube_marker_pose, self.paper_marker_pose]

        self.marker_publisher.publish(marker_pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

