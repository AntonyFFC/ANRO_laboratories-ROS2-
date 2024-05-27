import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray, Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np
from numpy import sin, cos

camera_effector_difference_x = 0.1
camera_effector_difference_z = 0

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.joint_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        self.m_subscriber = self.create_subscription(PoseArray, 'camera_link', self.pose_callback, 10)
        self.second_pose_subscriber = self.create_subscription(Bool, 'second_pose', self. sp_callback, 10)
        self.finish_subscriber = self.create_subscription(Bool, 'finished', self.finish_callback, 10)
        self.flag_subscriber = self.create_subscription(Bool , 'important_flag', self.flag_callback, 10)
        self.m_publisher = self.create_publisher(Marker, 'vizualization_marker', 10)
        self.finish_publisher = self.create_publisher(Bool, 'finished', 10)
        self.camera_position = []
        self.publish = True
        self.second_pose = False
        self.run_back = False

    def flag_callback(self, msg: Bool):
        if msg.data:
            self.publish = False
        else:
            self.publish = True

    def sp_callback(self, msg: Bool):
        if msg.data:
            self.second_pose = True
    
    def finish_callback(self, msg: Bool):
        if msg.data:
            self.second_pose = False
            self.publish_marker()

    def joint_states_callback(self ,msg: JointState):
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

    def pose_callback(self, msg: PoseArray):
        for i in range(2):
            
            x = msg.poses[i].position.x
            y = msg.poses[i].position.y
            z = msg.poses[i].position.z

            rx = msg.poses[i].orientation.x
            ry = msg.poses[i].orientation.y
            rz = msg.poses[i].orientation.z
            rw = msg.poses[i].orientation.w

            in_d = np.array([x, y, z, 1])

            if i == 0:
                self.cube_pos = np.matmul(self.tac, in_d.T)
                self.cube_rot = [rx, ry, rz, rw]
            else:
                self.paper_pos = np.matmul(self.tac, in_d.T)
                self.paper_rot = [rx, ry, rz, rw]

        self.publish_marker()

    def publish_marker(self):
        # new_msg = Bool()
        # new_msg.data = False
        # self.finish_publisher.publish(new_msg)
        marker1 = Marker()
        marker2 = Marker()
        marker1.header.frame_id = "base"
        marker1.header.stamp = self.get_clock().now().to_msg()
        
        marker1.id = 1
        marker1.type = Marker.CUBE
        marker1.action = Marker.ADD
        
        if self.second_pose:
            marker1.pose.position.x = self.paper_pos.item(0,0)
            marker1.pose.position.y = self.paper_pos.item(0,1)
            marker1.pose.position.z = self.cube_pos.item(0,2)
            marker1.pose.orientation.x = self.paper_rot[0]
            marker1.pose.orientation.y = self.paper_rot[1]
            marker1.pose.orientation.z = self.paper_rot[2]
            marker1.pose.orientation.w = self.paper_rot[3]
        else:
            marker1.pose.position.x = self.cube_pos.item(0,0)
            marker1.pose.position.y = self.cube_pos.item(0,1)
            marker1.pose.position.z = self.cube_pos.item(0,2)
            marker1.pose.orientation.x = self.cube_rot[0]
            marker1.pose.orientation.y = self.cube_rot[1]
            marker1.pose.orientation.z = self.cube_rot[2]
            marker1.pose.orientation.w = self.cube_rot[3]

        
        marker1.scale.x = 0.02
        marker1.scale.y = 0.02
        marker1.scale.z = 0.02
        
        marker1.color.r = 1.0
        marker1.color.g = 0.984
        marker1.color.b = 0.0
        marker1.color.a = 1.0
        
        if self.publish:
            self.m_publisher.publish(marker1)

        marker2.header.frame_id = "base"
        marker2.header.stamp = self.get_clock().now().to_msg()
        
        marker2.id = 2
        marker2.type = Marker.CUBE
        marker2.action = Marker.ADD
        
        marker2.pose.position.x = self.paper_pos.item(0,0)
        marker2.pose.position.y = self.paper_pos.item(0,1)
        marker2.pose.position.z = self.paper_pos.item(0,2)
        marker2.pose.orientation.x = self.paper_rot[0]
        marker2.pose.orientation.y = self.paper_rot[1]
        marker2.pose.orientation.z = self.paper_rot[2]
        marker2.pose.orientation.w = self.paper_rot[3]
        
        marker2.scale.x = 0.1
        marker2.scale.y = 0.2
        marker2.scale.z = 0.001

        marker2.color.r = 1.0
        marker2.color.g = 1.0
        marker2.color.b = 1.0
        marker2.color.a = 1.0

        self.m_publisher.publish(marker2)

def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerPublisher()
    rclpy.spin(marker_publisher)

    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()