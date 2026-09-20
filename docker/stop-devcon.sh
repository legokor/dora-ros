#!/bin/bash

# Stops the container and saves progress on the host.

echo "Stopping container:"
docker container stop dev
echo "Saving progress:"
docker container cp dev:/root/dora-ros/ros2_ws .
# Git operations. Plan is to add the commit message and branch as arguments to the script
# git commit -m "$1"
# git push origin HEAD:$2
# Docker does not provide any ways to automatically trigger scripts upon detaching from
# a container. Sadly, we will have to REMEMBER commiting and pushing.
# Hardest thing I've done in my life.
