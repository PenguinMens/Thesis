import os

import launch_ros.descriptions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare the launch arguments
    declared_arguments = [
        DeclareLaunchArgument('paused', default_value='true', description='Pause simulation'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('gui', default_value='true', description='Enable GUI'),
        DeclareLaunchArgument('headless', default_value='false', description='Run headless'),
        DeclareLaunchArgument('debug', default_value='false', description='Run in debug mode')
    ]

    # Get the package share directory
    gazebo_ros_share_dir = get_package_share_directory('gazebo_ros')
    example_package_share_dir = get_package_share_directory('robot_visual')

    # Include the empty_world launch file from gazebo_ros
    include_empty_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world_name': os.path.join(example_package_share_dir, 'world', 'example_track_all.world'),
            'debug': LaunchConfiguration('debug'),
            'gui': LaunchConfiguration('gui'),
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'headless': LaunchConfiguration('headless')
        }.items()
    )

    relay_nodes = [
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_relay',
            output='screen',
            parameters=[{'input_topic': '/cmd_vel', 'output_topic': '/example_track/sprocket_velocity_controller/command'}]
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_relay2',
            output='screen',
            parameters=[{'input_topic': '/cmd_vel', 'output_topic': '/example_track_simple/sprocket_velocity_controller/command'}]
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_relay3',
            output='screen',
            parameters=[{'input_topic': '/cmd_vel', 'output_topic': '/example_track_simple_wheels/sprocket_velocity_controller/command'}]
        )
    ]


    # Define the groups for each namespace
    def create_group(ns):
        return GroupAction([
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description':  launch_ros.descriptions.ParameterValue(Command(['xacro ', os.path.join(example_package_share_dir, 'urdf', 'example_track.urdf.xacro')]), value_type=str)}
                            ,{'tf_prefix': ns}],
                namespace=ns,
                
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                output='screen',
                arguments=['joint_state_controller', 'sprocket_velocity_controller'],
                namespace=ns
            )
        ])

    groups = [
        create_group('example_track'),
        create_group('example_track_simple'),
        create_group('example_track_simple_wheels')
    ]

    # Create the launch description
    ld = LaunchDescription(declared_arguments + [include_empty_world_launch] + relay_nodes + groups)

    return ld
