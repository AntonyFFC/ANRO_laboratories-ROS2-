from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    urdf_tutorial_path = FindPackageShare('urdf_tutorial')
    default_model_path = PathJoinSubstitution(['dobot.urdf.xacro'])
    default_rviz_config_path = PathJoinSubstitution([urdf_tutorial_path, 'rviz', 'dobot3.rviz'])

    # These parameters are maintained for backwards compatibility
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    # This parameter has changed its meaning slightly from previous versions
    ld.add_action(DeclareLaunchArgument(name='model', default_value=default_model_path,
                                        description='Path to robot urdf file relative to urdf_tutorial package'))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'urdf_tutorial',
            'urdf_package_path': LaunchConfiguration('model'),
            'jsp_gui': LaunchConfiguration('gui'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            # 'rviz_config': PathJoinSubstitution([urdf_tutorial_path, 'rviz', LaunchConfiguration('rvizconfig')]),
            }.items()

    ))

    ld.add_action(
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='rqt_plot_node',
            output='screen',
        )
    )

    # ld.add_action(
    #     Node(
    #         package='ForwardKin',
    #         executable='ForwardKin',
    #         name='ForwardKin_node',
    #     )
    # )

    ld.add_action(
        Node(
            package='rqt_graph',
            executable='rqt_graph',
            name='rqt_graph_node',
            output='screen',
        )
    )

    ld.add_action(
        Node(
            package='lab4PD',
            executable='reverse_kinematics',
            name='Reverse_Kinematics'
        )
    )

    # ld.add_action(
    #     Node(
    #         package='lab3',
    #         executable='lab3',
    #         name='komunikator'
    #     )
    # )

    # ld.add_action(
    #     Node(
    #         package='lab4',
    #         executable='Move_To_Point',
    #         name='Move_To_Point'
    #     )
    # )

    ld.add_action(
        Node(
            package='lab6PD',
            executable='marker_publisher',
            name='marker_publisher'
        )
    )

    ld.add_action(
        Node(
            package='lab6PD',
            executable='robot_pilot',
            name='robot_pilot'
        )
    )

    return ld
