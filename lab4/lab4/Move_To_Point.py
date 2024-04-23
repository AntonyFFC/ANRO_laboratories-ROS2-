import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from dobot_msgs.action import PointToPoint
# from dobot_msgs.srv import GripperControl
from sensor_msgs.msg import JointState
import numpy as np
from geometry_msgs.msg import PointStamped
from math import isnan

class Move_To_Point(Node):

    def __init__(self):
        super().__init__('Move_To_Point')
        self.subscription = self.create_subscription(PointStamped,'/clicked_point',self.clicked_point_callback,10)
        self.action = ActionClient(self, PointToPoint, 'PTP_action')

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

        move_goal = PointToPoint.Goal()
        move_goal.target_pose = [self.x,self.y,self.z]
        move_goal.motion_type = 1
        self.get_logger().info('Sending goal to move robot' )
        self.move_client.wait_for_server()
        move_future= self.move_client.send_goal_async(move_goal)
        move_future.add_done_callback(self.move_started_callback)


    def move_started_callback(self, future):
        move_outcome = future.result()
        if not move_outcome.accepted:
            self.get_logger().info('rejected goal')
            return
        self.get_logger().info('accepted goal')

def main(args=None):
    rclpy.init(args=args)
    client = Move_To_Point()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
