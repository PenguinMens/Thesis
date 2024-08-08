import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the path to the sllidar_ros2 launch file
    sllidar_launch_file = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'view_sllidar_a1_launch.py'
    )

    # Include the sllidar_ros2 launch file
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sllidar_launch_file)
    )
    return LaunchDescription([
        sllidar_launch
    ])