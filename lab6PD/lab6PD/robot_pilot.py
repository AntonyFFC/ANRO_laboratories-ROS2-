import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseArray
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from transforms3d.euler import quat2euler, euler2quat
from numpy import sin, cos
from time import sleep

camera_effector_difference_x = 0.1
camera_effector_difference_z = 0
frame_rate = 10

entry_x = 0.177
entry_y = 0.0
entry_z = 0.195

class RobotPilot(Node):
    def __init__(self):
        super().__init__('robot_pilot')

        self.set_entry_dobot_pose()
        self.joint_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        self.pose_array_subscriber = self.create_subscription(PoseArray, 'camera_link', self.point_callback, 10)
        self.pose_publisher = self.create_publisher(PointStamped, 'dobot_pose', 10)
        self.flag_publisher = self.create_publisher(Bool, 'important_flag', 10)
        self.second_pose_publisher = self.create_publisher(Bool, 'second_pose', 10)
        self.angle_publisher = self.create_publisher(JointState, 'tool_rot', 10)
        self.finish_publisher = self.create_publisher(Bool, 'finished', 10)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1 / frame_rate, self.timer_callback)

        self.point_1 = None
        self.point_3 = None
        self.publish_marker = False
        self.tool_rotation = 0.0

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
            result_matrix = np.matmul(joint_matrices[i], joint_matrices[i + 1])
            joint_matrices[i + 1] = result_matrix

        self.tab_matrix = joint_matrices[5]
        self.tbc_matrix = np.matrix([
            [1, 0, 0, camera_effector_difference_x],
            [0, 1, 0, 0],
            [0, 0, 1, -camera_effector_difference_z],
            [0, 0, 0, 1]
        ])
        self.tac_matrix = np.matmul(self.tab_matrix, self.tbc_matrix)

    def point_callback(self, msg: PoseArray):
        for i in range(2):
            x = msg.poses[i].position.x
            y = msg.poses[i].position.y
            z = msg.poses[i].position.z

            rx = msg.poses[i].orientation.x
            ry = msg.poses[i].orientation.y
            rz = msg.poses[i].orientation.z
            rw = msg.poses[i].orientation.w

            camera_coordinates = np.array([x, y, z, 1])
            robot_coordinates = np.matmul(self.tac_matrix, camera_coordinates.T)

            x_robot = robot_coordinates.item(0, 0)
            y_robot = robot_coordinates.item(0, 1)
            z_robot = robot_coordinates.item(0, 2)

            if i == 1:
                self.point_1 = [x_robot, y_robot, z_robot + 0.1]
                self.point_2 = [x_robot, y_robot, z_robot + 0.03]
                self.paper_rotation = [rw, rx, ry, rz]
            else:
                self.point_3 = [x_robot, y_robot, z_robot + 0.1]
                self.point_4 = [x_robot, y_robot, z_robot + 0.03]
                self.cube_rotation = [rw, rx, ry, rz]

    def set_entry_dobot_pose(self):
        self.current_pose = PointStamped()
        self.current_pose.header.frame_id = "base"
        self.current_pose.point.x = entry_x
        self.current_pose.point.y = entry_y
        self.current_pose.point.z = entry_z

    def timer_callback(self):
        self.pose_publisher.publish(self.current_pose)

        if self.publish_marker:
            marker = Marker()
            marker.header.frame_id = "base"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = 1
            marker.type = Marker.CUBE

            marker.pose.position.x = self.current_pose.point.x
            marker.pose.position.y = self.current_pose.point.y
            marker.pose.position.z = self.current_pose.point.z - 0.021

            cube_quat = euler2quat(0.0, 0.0, self.tool_rotation)

            marker.pose.orientation.x = cube_quat[1]
            marker.pose.orientation.y = cube_quat[2]
            marker.pose.orientation.z = cube_quat[3]
            marker.pose.orientation.w = cube_quat[0]

            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02

            marker.color.r = 1.0
            marker.color.g = 0.984
            marker.color.b = 0.0
            marker.color.a = 1.0

            self.marker_publisher.publish(marker)

    def operation(self):
        self.published = False
        while (self.point_1 is None or self.point_3 is None) and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1 / frame_rate)

        # 1) General move to the middle
        self.move_to_point([0.2, 0, 0.15, 1], False)

        # 2) Move above the box
        self.move_to_point(self.point_3, False)

        # 3) Rotate tool
        self.rotate_tool(self.cube_rotation)

        # 4) Move down to the box
        self.move_to_point(self.point_4, False)

        # 5) Move up
        self.move_to_point(self.point_3, True)

        # 6) Move above the paper
        self.move_to_point(self.point_1, True)

        # 7) Rotate tool
        self.rotate_tool(self.paper_rotation)

        # 8) Move down to the paper
        self.move_to_point(self.point_2, True)
        msg = Bool()
        msg.data = True
        self.second_pose_publisher.publish(msg)

        # 9) Move up
        self.move_to_point(self.point_1, False)

        # 10) Return to the entry pose
        return_pose = [entry_x, entry_y, entry_z]
        self.move_to_point(return_pose, False)
        self.rotate_tool([1, 0, 0, 0])
        msg = Bool()
        msg.data = True
        self.finish_publisher.publish(msg)
    
    def rotate_tool(self, quat):
        _, _, yaw = quat2euler(quat)

        self.tool_rotation = yaw
        self.get_logger().info(f"Tool rotation (yaw): {yaw}")

        joint_states = JointState()
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = ['effector_rot_joint']
        joint_states.position = [yaw]
        self.angle_publisher.publish(joint_states)

    def move_to_point(self, point, move_marker):
        self.publish_marker = move_marker
        flag_msg = Bool()
        flag_msg.data = move_marker
        self.flag_publisher.publish(flag_msg)

        dx = point[0] - self.current_pose.point.x
        dy = point[1] - self.current_pose.point.y
        dz = point[2] - self.current_pose.point.z

        dist = np.sqrt(dx**2 + dy**2 + dz**2)
        move_time = dist / 0.01
        
        steps = max(int(move_time * frame_rate), 1)

        x_step = dx / steps
        y_step = dy / steps
        z_step = dz / steps

        for _ in range(steps):
            self.current_pose.point.x += x_step
            self.current_pose.point.y += y_step
            self.current_pose.point.z += z_step
            rclpy.spin_once(self, timeout_sec=1 / frame_rate)

def main(args=None):
    rclpy.init(args=args)
    node = RobotPilot()
    for _ in range(10):
        sleep(3)
        node.operation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
