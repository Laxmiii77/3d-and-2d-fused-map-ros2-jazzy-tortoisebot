from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='depth_to_scan',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': -0.5,
                'max_height': 2.0,
                'angle_min': -1.5708,
                'angle_max': 1.5708,
                'angle_increment': 0.0087,
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
                # Force RELIABLE QoS to match RViz expectation
                'qos_overrides': {
                    '/depth_scan': {
                        'reliability': 'reliable',
                        'durability': 'volatile',
                        'history': 'keep_last',
                        'depth': 10
                    }
                }
            }],
            remappings=[
                ('cloud_in', '/depth_camera/points'),
                ('scan', '/depth_scan'),
            ],
        )
    ])