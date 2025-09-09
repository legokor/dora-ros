#!/bin/bash

DORA_PRE_RUN_PWD=$PWD

source /root/dora-ros/scripts/build.sh &&
cd /root/dora-ros/ros2_ws/ &&
source /root/dora-ros/ros2_ws/src/install/setup.bash &&
ros2 launch controller launch_dora.xml

cd $DORA_PRE_RUN_PWD

