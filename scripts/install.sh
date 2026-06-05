#!/bin/bash

## This script installs runtime packages

DORA_PRE_BUILD_PWD=$PWD

apt-get update && \
apt-get upgrade -y && \
\
apt-get install -y \
    # General apps and python
    software-properties-common nano curl btop tree unzip \
    python3 python3-pip \
    # Removed ranger and neovim to boost build time
    # Ros tools
    python3-rosdep \
    ros-dev-tools \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    # rplidar package is not maintained :/ \
    # ros-${ROS_DISTRO}-rplidar-ros \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox

# ros copy workspace
cd /opt/ros && git clone --depth=1 https://github.com/legokor/dora-ros.git

# RPLIDAR
cd /opt/ros/dora-ros/ros2_ws/src/ && git clone --depth=1 -b ros2 https://github.com/Slamtec/rplidar_ros.git