#!/bin/bash

# Removes the devcontainer from the computer
# This script doesn't remove the image
# If there are changes in the image, docker compose up automatically rebuilds

echo "Removing container and volume: (Takes a bit of time to gracefully stop)"
docker container stop dev
docker container rm dev
docker volume rm docker_dev-vol
echo "If no error messages were present, then removing was a success!"
