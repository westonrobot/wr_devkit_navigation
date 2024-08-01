#!/bin/bash

packages=(
    ros-dev-tools
    ros-humble-joint-state-publisher
    ros-humble-pointcloud-to-laserscan
    ros-humble-cartographer-ros
    ros-humble-nav2-map-server
    ros-humble-navigation2
    ros-humble-nav2-bringup
    ros-humble-xacro
    ros-humble-librealsense2*
    ros-humble-realsense2-*
    ros-humble-rtabmap-slam
    ros-humble-rtabmap-viz
)

sudo apt update
sudo apt install -y "${packages[@]}"