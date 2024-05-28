import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import GripperControl

class WiezaClient(Node):

    def __init__(self):
        super().__init__("wieza")
        self.move_client = ActionClient(self, PointToPoint, 'PTP_action', callback_group = ReentrantCallbackGroup())
        self.client = self.create_client(GripperControl, 'dobot_gripper_service', callback_group = ReentrantCallbackGroup())
        self.grip = GripperControl.Request()
    
    def move_robot(self, goal_position, goal_type):
        move_goal = PointToPoint.Goal()
        move_goal.target_pose = goal_position
        move_goal.motion_type = goal_type
        self.get_logger().info('Sending goal to move robot' )
        self.move_client.wait_for_server()
        move_future= self.move_client.send_goal_async(move_goal)
        move_future.add_done_callback(self.move_started_callback)
        return move_future

    def move_started_callback(self, future):
        move_outcome = future.result()
        if not move_outcome.accepted:
            self.get_logger().info('rejected goal')
            return
        self.get_logger().info('accepted goal')

    def grip_start(self, state):
        self.is_moving = True
        self.grip.gripper_state = state
        self.grip.keep_compressor_running = False
        self.get_logger().info('Sending grip request to gripper '+ state)
        self.srv_future = self.client.call_async(self.grip)
        # self.timer = self.create_timer(0.1, self.timer_callback, callback_group = ReentrantCallbackGroup())
    
    # def timer_callback(self):
    #     if self.srv_future.done():
    #         self.timer.cancel()
    #         return

    
def main(args=None):
    rclpy.init(args=args)
    client = WiezaClient()
    moveX=30;moveY=40;moveZ=55
    x = 150.0
    y = -6.0
    z = 6.0
    client.move_robot([x,y,z-moveZ,0.0],1)
    # client.move_robot([x,y,z,0.0],1)
    # client.grip_start("open")
    # for i in range(3):
    #     client.move_robot([x,y,z,0.0],1)
    #     client.move_robot([x,y,z-moveZ,0.0],1)
    #     client.grip_start("close")
    #     client.move_robot([x,y,z,0.0],1)
    #     client.move_robot([x+moveX,y,z,0.0],1)
    #     client.move_robot([x+moveX,y,z-moveZ,0.0],1)
    #     client.grip_start("open")
    #     client.move_robot([x+moveX,y,z,0.0],1)
    #     x -=25
    #     moveZ -= 20
    #     moveX +=25



    executor = MultiThreadedExecutor()
    rclpy.spin(client, executor)

if __name__ == '__main__':
    main()
