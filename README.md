# 🌐 2D–3D Fused Mapping with ROS 2 Jazzy using OctoMap

This repository implements sensor-fused mapping using a 2D LiDAR and a depth camera to generate a 3D OctoMap along with a 2D projected map in ROS 2 Jazzy Jalisco.
The system combines planar laser scans and 3D point clouds to build a consistent volumetric representation of the environment.

## Simulation Video
https://www.youtube.com/watch?v=1SAAv57ydL4

<image src=image.png/>

## 🚀 Key Features

🔄 Fusion of 2D LiDAR + Depth Camera
🧱 3D occupancy mapping using OctoMap
🗺️ 2D map projection from 3D OctoMap
📡 Works in simulation (Gazebo) and real robots
🧭 Ready for Nav2 & obstacle-aware navigation

## 🧰 Tech Stack
ROS 2 Jazzy Jalisco
OctoMap
SLAM Toolbox (2D pose estimation)
Depth Camera (RGB-D / Stereo)
2D LiDAR
Gazebo
RViz2

## 📦 Installation
```bash
sudo apt update
sudo apt install ros-jazzy-octomap ros-jazzy-octomap-ros ros-jazzy-depthimage-to-laserscan
```
## Clone and build the workspace:
```bash
cd ros2_ws/src
git clone https://github.com/Laxmiii77/3d-and-2d-fused-map-ros2-jazzy-tortoisebot
cd ..
colcon build
source install/setup.bash
```
## Launch the robot in the custom world
```bash
ros2 launch my_robot_description robot_gazebo.launch.py
```
## Run the node to convert Laserscan data to Pointcloud2
```bash
ros2 run my_robot_description conversion
```
## Run the fusion node
```bash
ros2 run my_robot_description fusion
```
## Run Rviz2
```bash
rviz2
```
## Launch octomap 
```bash
ros2 launch my_robot_description octomap.launch.py
```
