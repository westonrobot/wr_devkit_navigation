#!/bin/bash

packages=(
    ros-dev-tools
    ros-humble-joint-state-publisher
    ros-humble-pointcloud-to-laserscan
    ros-humble-cartographer-ros
    ros-humble-nav2-map-server
    ros-humble-navigation2
    ros-humble-nav2-bringup
)

sudo apt update
sudo apt install -y "${packages[@]}"