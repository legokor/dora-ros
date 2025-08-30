#!/bin/bash

# build our packages
cd /root/dora-ros/ros2_ws/src/ &&
source /opt/ros/${ROS_DISTRO}/setup.bash &&
colcon build

cd /root/dora-ros/ros2_ws/ &&
source /root/dora-ros/ros2_ws/src/install/setup.bash &&
ros2 launch controller launch_dora.xml

