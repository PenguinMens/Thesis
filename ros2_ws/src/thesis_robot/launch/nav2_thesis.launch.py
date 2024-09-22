from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'min_particles': 500,
                'max_particles': 5000,
                'initial_pose_x': 0.0,
                'initial_pose_y': 0.0,
                'initial_pose_a': 0.0,
                'update_min_d': 0.2,
                'update_min_a': 0.2,
            }],
            remappings=[
                ('/scan', '/scan'),
                ('/map', '/map'),
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ]
        ),
        Node(
            package='nav2_bringup',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': '/home/gabriel/map.yaml'}],
        ),
        Node(
            package='nav2_bringup',
            executable='lifecyle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
    ])
