#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get paths
    world_path = "/home/laxmi-arz-i006/map_with_lidar/ros2_ws/src/my_robot_description/worlds/empty_world_with_obstacles.sdf"
    sdf_path = "/home/laxmi-arz-i006/map_with_lidar/ros2_ws/src/my_robot_description/sdf/my_robot.sdf"
    
    # Load SDF contents for robot description
    with open(sdf_path, 'r') as sdf_file:
        robot_description = sdf_file.read()

    # Launch Gazebo with the world file
    gazebo_launch = ExecuteProcess(
        cmd=['gz', 'sim', world_path, '-v', '4'],
        output='screen'
    )

    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'publish_frequency': 50.0
        }],
        output='screen'
    )

    # Spawn robot into the running Gazebo world (delayed to ensure Gazebo is ready)
    spawn_entity = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'tortoisebot',
                '-file', sdf_path,
                '-x', '0.0',
                '-y', '0.0', 
                '-z', '0.2'
            ],
            output='screen'
        )]
    )

    # Enhanced ROS-Gazebo bridge with better topic mappings
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Add model-specific joint states bridging
            '/model/tortoisebot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera/image@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',

        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Static transform publishers for sensor frames
    static_transform_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'tortoisebot/lidar_link/lidar_sensor'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Additional static transforms to ensure proper frame connectivity
    # This helps bridge any gap between Gazebo's internal frames and ROS frames
    static_transform_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'tortoisebot/base_link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_pub,
        spawn_entity,
        bridge_node,
        static_transform_lidar,
        static_transform_base
    ])