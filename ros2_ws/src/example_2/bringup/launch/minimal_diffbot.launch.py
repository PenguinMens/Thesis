# Filename: test_diffbot_system_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ros2_control_demo_example_2"), "urdf", "diffbot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Only the controller manager (ros2_control_node) for testing the hardware interface
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description],
        output="screen",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    return LaunchDescription([control_node])
