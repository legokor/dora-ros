#!/bin/bash

## This script installs runtime packages

DORA_PRE_BUILD_PWD=$PWD

apt-get update && \
apt-get upgrade -y && \
\
apt-get install -y \
    python3-rosdep \
    ros-dev-tools \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox
    # rplidar package is not maintained :/ \
    # ros-${ROS_DISTRO}-rplidar-ros 
# ros copy workspace
if ! [ -d "/opt/ros/dora-ros" ]; then
    cd /opt/ros && git clone -b Nav2_feature --depth=1 https://github.com/legokor/dora-ros.git
    # RPLIDAR
    cd /opt/ros/dora-ros/ros2_ws/src/ && git clone --depth=1 -b ros2 https://github.com/Slamtec/rplidar_ros.git
fi

cd $DORA_PRE_BUILD_PWD