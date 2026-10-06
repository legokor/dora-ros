#!/bin/bash

## This script installs runtime packages outside container building
# Volumes keep downloaded packages, so this script should be ran once when creating new volumes 

# ROS packages and dev tools
packages =(
	software-properties-common nano curl btop tree unzip neovim \
    python3 python3-pip
    python3-rosdep \  # automatic ROS dependency installer
    ros-dev-tools \  # Dev tools
    ros-${ROS_DISTRO}-xacro \  # XML macro runner for URDF files.
    ros-${ROS_DISTRO}-joint-state-publisher \  # Publishes join states like how many degrees a wheel has rotated
    # rplidar package is not maintained :/ \
    # ros-${ROS_DISTRO}-rplidar-ros
)
apt update
apt upgrade -y
apt install -y "${packages[@]}"
	
