#!/bin/bash

DORA_PRE_BUILD_PWD=$PWD

cd /root/dora-ros/ros2_ws/ &&
source /opt/ros/${ROS_DISTRO}/setup.bash &&
rosdep install --from-paths src -y --ignore-src
colcon build

cd $DORA_PRE_BUILD_PWD

