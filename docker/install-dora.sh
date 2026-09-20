#!/bin/bash

## This script installs runtime packages outside container building
# Volumes keep downloaded packages, so this script should be ran once when creating new volumes 

# ROS packages and dev tools
apt install -y \
    python3-rosdep \  # automatic ROS dependency installer
    ros-dev-tools \  # Dev tools
    ros-${ROS_DISTRO}-xacro \  # XML macro runner for URDF files.
    ros-${ROS_DISTRO}-joint-state-publisher \  # Publishes join states like how many degrees a wheel has rotated
    ros-${ROS_DISTRO}-navigation2 \  # Nav2 packages
    ros-${ROS_DISTRO}-nav2-bringup \  # Launch files for Nav2
    ros-${ROS_DISTRO}-slam-toolbox \  # SLAM packages
    # rplidar package is not maintained :/ \
    # ros-${ROS_DISTRO}-rplidar-ros
