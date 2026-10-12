#!/bin/bash

# Removes the robot's container
# This script doesn't remove the container's image
# If there are changes in the image, docker compose up automatically rebuilds

## Flags
# -v : don't remove volumes
# -c : don't remove container

rm_volumes=true
rm_container=true
while [[ $# -gt 0 ]]; do
	case $1 in
		-v) rm_volumes=false;;
		-c) rm_container=false;;
		*)
	esac
	shift
done

if rm_volumes; then
	echo "Removing container: (Takes a bit of time to gracefully remove)"
	docker compose -f docker/docker-compose.yml down dora
fi

if rm_container; then
	echo "Removing volumes:"
	docker volume rm docker_dora-vol
fi

echo "Check if docker had output to confirm success."

