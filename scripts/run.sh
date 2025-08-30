#!/bin/bash

source /root/dora-ros/scripts/build.sh &&
cd /root/dora-ros/ros2_ws/ &&
source /root/dora-ros/ros2_ws/src/install/setup.bash &&
ros2 launch controller launch_dora.xml

