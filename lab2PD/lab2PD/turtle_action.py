import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import turtlesim.action._rotate_absolute
from turtlesim.srv import Spawn
import math
from time import sleep

class TurtleAction(Node):
    def __init__(self):
        super().__init__('turtle_action')
        self.spawn_client = self.create_client(turtlesim.srv.Spawn, 'spawn')
        self.rotation_client = ActionClient(self, turtlesim.action.RotateAbsolute, 'turtle1/rotate_absolute')
        self.was_spawned = False
        self.is_moving = False
        
    def rotate_turtle(self, angle):
        self.is_moving = True
        self.get_logger().info('Sending goal to rotate turtle by %d degrees' % (angle))
        rotation_goal = turtlesim.action.RotateAbsolute.Goal(theta=math.radians(angle))
        self.rotation_client.wait_for_server()
        rotation_future= self.rotation_client.send_goal_async(rotation_goal)
        rotation_future.add_done_callback(self.rotation_started_callback)
        return rotation_future

    def rotation_started_callback(self, future):
        rotation_outcome = future.result()
        if rotation_outcome.accepted:
            self.get_logger().info('Rotation started succesfuly')
            self.rotation_future = rotation_outcome.get_result_async()
            self.rotation_future.add_done_callback(self.rotated_callback)
        else:
            self.get_logger().info('Rotation failed to start')

    def spawn_turtle(self):
        self.get_logger().info('Sending goal to spawn new turtle')
        if self.was_spawned:
            name = "Antek"
            x = 5.0
        else:
            name = "Adrian"
            x = 3.0
        request = turtlesim.srv.Spawn.Request(name=name, x=x, y=8.0)
        self.spawn_client.wait_for_service()
        spawn_future = self.spawn_client.call_async(request)
        spawn_future.add_done_callback(self.spawned_callback)

    def spawned_callback(self, future):
        self.get_logger().info('turtle added')
        self.was_spawned = not self.was_spawned
        self.is_moving = False
    
    def rotated_callback(self, future):
        self.spawn_turtle()



def main(args=None):
    rclpy.init(args=args)
    turtle_action = TurtleAction()

    turtle_action.rotate_turtle(-180)
    while turtle_action.is_moving:
        rclpy.spin_once(turtle_action)
    sleep(1)

    turtle_action.rotate_turtle(270)
    while turtle_action.is_moving:
        rclpy.spin_once(turtle_action)

if __name__ == '__main__':
    main()
