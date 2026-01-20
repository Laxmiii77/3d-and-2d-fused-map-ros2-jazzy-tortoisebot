#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'resolution',
            default_value='0.1',
            description='OctoMap resolution'
        ),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value='odom',
            description='Frame for OctoMap'
        ),
        
        # Your point cloud fusion node
        Node(
            package='my_robot_description',  # Replace with your actual package name
            executable='fusion',
            name='pointcloud_fusion_fast',
            output='screen',
            parameters=[
            {'use_sim_time': True},

            ],
            remappings=[
                # Add any topic remappings if needed
            ]
        ),
        
        # OctoMap server node
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[
                {'resolution': LaunchConfiguration('resolution')},
                {'frame_id': LaunchConfiguration('frame_id')},
                {'use_sim_time': True},
                {'sensor_model/max_range': 10.0},
                {'sensor_model/hit': 0.9},
                {'sensor_model/miss': 0.4},
                {'sensor_model/min': 0.12},
                {'sensor_model/max': 0.97},
                {'latch': False},
                {'publish_free_space': False},
            ],
            remappings=[
                ('cloud_in', '/fused_pointcloud'),  # Subscribe to filtered cloud
            ]
        ),
    ])