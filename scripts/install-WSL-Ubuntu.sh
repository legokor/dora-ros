#!/bin/bash

## Helper script to install ros2 on native ubuntu and wsl
# Run from project root dir.
# Don't call with sudo because the ROS_DISTRO env-var will be set to root instead of the user

# Ros version:
export ROS_DISTRO=lyrical

# In case they are missing.
basePackages=(
	software-properties-common nano curl btop tree unzip
    python3 python3-pip
)

packages=(
	python3-rosdep  # automatic ROS dependency installer
	python3-pygame # for a prototype-teleop interface
    ros-dev-tools   # Dev tools
    ros-${ROS_DISTRO}-desktop   # Ros2 for developtment
    ros-${ROS_DISTRO}-xacro   # XML macro runner for URDF files.
    ros-${ROS_DISTRO}-joint-state-publisher   # Publishes join states like how many degrees a wheel has rotated
    ros-${ROS_DISTRO}-robot-state-publisher   # Publishes join states like how many degrees a wheel has rotated
    ros-${ROS_DISTRO}-navigation2   # Nav2 packages
    ros-${ROS_DISTRO}-nav2-bringup   # Launch files for Nav2
    ros-${ROS_DISTRO}-slam-toolbox   # SLAM packages
    ros-${ROS_DISTRO}-rviz2   # Rviz2
    ros-${ROS_DISTRO}-rviz-default-plugins   # Helpful plugins
    ros-${ROS_DISTRO}-rqt   # Analytics panel for ROS data
    ros-${ROS_DISTRO}-rqt-common-plugins  # Better logging, settings etc...
    ros-${ROS_DISTRO}-joint-state-publisher-gui   # Allows manual joint configuration
    ros-${ROS_DISTRO}-turtlesim
    # rplidar package is not maintained :/ 
    # ros-${ROS_DISTRO}-rplidar-ros
)

# Installing base packages
sudo apt update
sudo apt upgrade -y
sudo apt install -y "${basePackages[@]}"

# Setting up ros install:
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Installing
sudo apt update
sudo apt install ros-dev-tools "${packages[@]}"

# Sourcing bash to apply downloaded theme to QT via environment vars
source ./dora-ros/scripts/bashrcExtension.bash


