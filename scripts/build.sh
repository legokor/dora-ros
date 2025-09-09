#!/bin/bash

DORA_PRE_BUILD_PWD=$PWD

cd /root/dora-ros/ros2_ws/src/ &&
source /opt/ros/${ROS_DISTRO}/setup.bash &&
colcon build

cd DORA_PRE_BUILD_PWD

