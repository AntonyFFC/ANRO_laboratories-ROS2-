from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    urdfPath = os.path.join(
        get_package_share_directory('lab3PD'),
        'dobot.urdf.xacro')
    valuesPath = os.path.join(
        get_package_share_directory('lab3PD'),
        'values.yaml')
    # rvizPath = os.path.join(
    #     get_package_share_directory('lab3PD'),
    #     'dobot.rviz')
    
    with open(urdfPath, 'r') as infu:
        dobot_desc= infu.read()
    
    with open(valuesPath, 'r') as infv:
        params = yaml.safe_load(infv)
    
    l0 = float(params['link']['l0'])
    l0_half = l0 / 2
    l1 = float(params['link']['l1'])
    l1_half = l0 / 2
    l2 = float(params['link']['l2'])
    l2_half = l0 / 2
    l3 = float(params['link']['l3'])
    l3_half = l0 / 2
    l4 = float(params['link']['l4'])
    l4_half = l0 / 2
    l5 = float(params['link']['l5'])
    l5_half = l0 / 2

    print(l0, l1, l2, l3, l4, l5, l0_half, l1_half, l2_half, l3_half, l4_half, l5_half)


    return LaunchDescription([
        DeclareLaunchArgument(
            'plot',
            default_value='false',
            choices=["true", "false"],
            description='Launch plot if true'),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': dobot_desc
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            #arguments=['-d', rvizPath]
        )

    ])
