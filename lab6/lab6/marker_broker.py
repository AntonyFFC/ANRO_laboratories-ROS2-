import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
from ros2_aruco_interfaces.msg import ArucoMarkers
from std_msgs.msg import Bool
import numpy as np
from numpy import sin, cos

camera_effector_difference_x = 0.1
camera_effector_difference_z = 0

class MarkerBroker(Node):
    def __init__(self):
        super().__init__('marker_broker')
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.joint_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        self.marker_subscriber = self.create_subscription(ArucoMarkers, 'aruco_markers', self.pose_callback, 10)
        self.flag_subscriber = self.create_subscription(Bool , 'important_flag', self.flag_callback, 10)   
        self.camera_position = []
        self.publish = True
        self.paper_position = None
        self.cube_position = None

    def flag_callback(self, msg: Bool):
        self.publish = not msg.data

    def joint_states_callback(self, msg: JointState):
        theta_vector = msg.position
        joint_matrices = []

        joint_matrices.append(np.matrix([
            [cos(theta_vector[0]), -sin(theta_vector[0]), 0, 0],
            [sin(theta_vector[0]), cos(theta_vector[0]), 0, 0],
            [0, 0, 1, 0.05],
            [0, 0, 0, 1]
        ]))

        joint_matrices.append(np.matrix([
            [cos(theta_vector[1]), 0, sin(theta_vector[1]), 0],
            [0, 1, 0, 0],
            [-sin(theta_vector[1]), 0, cos(theta_vector[1]), 0.088],
            [0, 0, 0, 1]
        ]))

        joint_matrices.append(np.matrix([
            [cos(theta_vector[2]), 0, sin(theta_vector[2]), 0],
            [0, 1, 0, 0],
            [-sin(theta_vector[2]), 0, cos(theta_vector[2]), 0.135],
            [0, 0, 0, 1]
        ]))

        joint_matrices.append(np.matrix([
            [cos(theta_vector[3]), 0, sin(theta_vector[3]), 0.147],
            [0, 1, 0, 0],
            [-sin(theta_vector[3]), 0, cos(theta_vector[3]), 0],
            [0, 0, 0, 1]
        ]))

        joint_matrices.append(np.matrix([
            [cos(theta_vector[4]), -sin(theta_vector[4]), 0, 0.03],
            [sin(theta_vector[4]), cos(theta_vector[4]), 0, 0],
            [0, 0, 1, -0.069],
            [0, 0, 0, 1]
        ]))

        joint_matrices.append(np.matrix([
            [1, 0, 0, 0],
            [0, cos(np.pi), -sin(np.pi), 0],
            [0, sin(np.pi), cos(np.pi), 0],
            [0, 0, 0, 1]
        ]))

        for i in range(len(theta_vector)):
            R = np.matmul(joint_matrices[i], joint_matrices[i + 1])
            joint_matrices[i + 1] = R

        self.tab_matrix = joint_matrices[5]
        self.tbc_matrix = np.matrix([
            [1, 0, 0, camera_effector_difference_x],
            [0, 1, 0, 0],
            [0, 0, 1, -camera_effector_difference_z],
            [0, 0, 0, 1]
        ])
        self.tac_matrix = np.matmul(self.tab_matrix, self.tbc_matrix)

    def pose_callback(self, msg: ArucoMarkers):
        for idx, num in enumerate(msg.marker_ids):
            x_pos = msg.poses[idx].position.x
            y_pos = msg.poses[idx].position.y
            z_pos = msg.poses[idx].position.z

            x_orient = msg.poses[idx].orientation.x
            y_orient = msg.poses[idx].orientation.y
            z_orient = msg.poses[idx].orientation.z
            w_orient = msg.poses[idx].orientation.w

            input_data = np.array([x_pos, y_pos, z_pos, 1])

            if num == 14: # Kostka QR
                self.cube_position = np.matmul(self.tac_matrix, input_data.T)
                self.cube_orientation = [x_orient, y_orient, z_orient, w_orient]
            elif num == 5: # Papier QR
                self.paper_position = np.matmul(self.tac_matrix, input_data.T)
                self.paper_orientation = [x_orient, y_orient, z_orient, w_orient]

        if (self.cube_position is not None) | (self.paper_position is not None):   
            self.publish_marker()
            self.paper_position = None
            self.cube_position = None


    def publish_marker(self):
        marker_cube = Marker()
        marker_paper = Marker()

        if self.cube_position is not None:
            marker_cube.header.frame_id = "base"
            marker_cube.header.stamp = self.get_clock().now().to_msg()
            marker_cube.id = 14
            marker_cube.type = Marker.CUBE
            marker_cube.action = Marker.ADD

            marker_paper.pose.position.x = self.cube_position.item(0, 0)
            marker_paper.pose.position.y = self.cube_position.item(0, 1)
            marker_paper.pose.position.z = self.cube_position.item(0, 2)
            marker_paper.pose.orientation.x = self.cube_orientation[0]
            marker_paper.pose.orientation.y = self.cube_orientation[1]
            marker_paper.pose.orientation.z = self.cube_orientation[2]
            marker_paper.pose.orientation.w = self.cube_orientation[3]

            marker_cube.scale.x = 0.02
            marker_cube.scale.y = 0.02
            marker_cube.scale.z = 0.02
            marker_cube.color.r = 1.0
            marker_cube.color.g = 0.984
            marker_cube.color.b = 0.0
            marker_cube.color.a = 1.0

            self.marker_publisher.publish(marker_cube)
        
        if self.paper_position is not None:
            marker_paper.header.frame_id = "base"
            marker_paper.header.stamp = self.get_clock().now().to_msg()
            marker_paper.id = 5
            marker_paper.type = Marker.CUBE
            marker_paper.action = Marker.ADD

            marker_paper.pose.position.x = self.paper_position.item(0, 0)
            marker_paper.pose.position.y = self.paper_position.item(0, 1)
            marker_paper.pose.position.z = self.paper_position.item(0, 2)
            marker_paper.pose.orientation.x = self.paper_orientation[0]
            marker_paper.pose.orientation.y = self.paper_orientation[1]
            marker_paper.pose.orientation.z = self.paper_orientation[2]
            marker_paper.pose.orientation.w = self.paper_orientation[3]

            marker_paper.scale.x = 0.02
            marker_paper.scale.y = 0.02
            marker_paper.scale.z = 0.02
            marker_paper.color.r = 1.0
            marker_paper.color.g = 0.984
            marker_paper.color.b = 0.0
            marker_paper.color.a = 1.0

            self.marker_publisher.publish(marker_paper)

        marker_paper.header.frame_id = "base"
        marker_paper.header.stamp = self.get_clock().now().to_msg()
        marker_paper.id = 2
        marker_paper.type = Marker.CUBE
        marker_paper.action = Marker.ADD

        marker_paper.pose.position.x = self.paper_position.item(0, 0)
        marker_paper.pose.position.y = self.paper_position.item(0, 1)
        marker_paper.pose.position.z = self.paper_position.item(0, 2)
        marker_paper.pose.orientation.x = self.paper_orientation[0]
        marker_paper.pose.orientation.y = self.paper_orientation[1]
        marker_paper.pose.orientation.z = self.paper_orientation[2]
        marker_paper.pose.orientation.w = self.paper_orientation[3]

        marker_paper.scale.x = 0.1
        marker_paper.scale.y = 0.2
        marker_paper.scale.z = 0.001
        marker_paper.color.r = 1.0
        marker_paper.color.g = 1.0
        marker_paper.color.b = 1.0
        marker_paper.color.a = 1.0

        self.marker_publisher.publish(marker_paper)

def main(args=None):
    rclpy.init(args=args)
    marker_broker = MarkerBroker()
    rclpy.spin(marker_broker)

    marker_broker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
