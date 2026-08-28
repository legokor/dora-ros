#!/bin/bash

# Removes the robot's container from the computer
# This script doesn't remove the image
# If there are changes in the image, docker compose up automatically rebuilds

echo "Removing container and volume: (Takes a bit of time to gracefully remove)"
docker compose down dora
docker volume rm docker_dora-vol
echo "If no error messages were present, then removing was a success!"
