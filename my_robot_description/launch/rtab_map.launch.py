from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Adjust this to your actual package name
    package_dir = get_package_share_directory('my_robot_description')
    config_file = os.path.join(package_dir, 'config', 'rtabmap_config.yaml')

    return LaunchDescription([

        # RTAB-Map SLAM node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                config_file,
                {"use_sim_time": True,
                 "subscribe_scan_cloud": True,
                 "subscribe_scan": False,
                 "subscribe_depth": False,
                 "subscribe_rgb": False,
                 "subscribe_rgbd": False,
                 "subscribe_odom_info": False,
                 "approx_sync": True}
            ],
            remappings=[
                ("scan_cloud", "/fused_pointcloud"),
                ("odom", "/odom"),
            ],
            arguments=["--delete_db_on_start"]
        ),

        # RTAB-Map visualization node
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[
                {"use_sim_time": False,
                 "subscribe_scan_cloud": True,
                 "subscribe_scan": False,
                 "subscribe_depth": False,
                 "subscribe_rgb": False,
                 "subscribe_rgbd": False,
                 "subscribe_odom_info": False,
                 "approx_sync": True}
            ],
            remappings=[
                ("scan_cloud", "/fused_pointcloud"),
                ("odom", "/odom"),
            ]
        ),
    ])
