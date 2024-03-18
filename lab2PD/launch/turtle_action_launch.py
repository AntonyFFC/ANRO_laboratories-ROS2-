from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turt',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='lab2PD',
            executable='turtle_action',
            namespace='turt',
            name='turt_act',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'name2': 'Tomek',
                 'name1': 'Piotr'}
            ]
        )
    ])