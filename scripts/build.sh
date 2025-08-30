#!/bin/bash

cd /root/dora-ros/ros2_ws/src/ &&
source /opt/ros/${ROS_DISTRO}/setup.bash &&
colcon build

