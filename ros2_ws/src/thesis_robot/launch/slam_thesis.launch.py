from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='online_async_launch.py',
            name='slam_toolbox',
            output='screen',
            parameters=['./src/thesis_robot/config/mapper_params_online_async.yaml'],
            arguments=['--ros-args', '--params-file', './src/thesis_robot/config/mapper_params_online_async.yaml'],
            remappings=[
                # You can remap topics if needed
            ],
            # Other arguments you need to pass can be specified here
        ),
    ])
