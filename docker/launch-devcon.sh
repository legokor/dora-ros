#!/bin/bash

# Run from root directory with "docker/launch-devcon.sh" !

# Enabling X11 forwarding support. 
xhost +local:docker

# If you want to force rebuild, run the script with -f flag
if [[ "$1" == "-f" ]]; then
	docker compose -f docker/docker-compose.yml up --build -d dev
else
	docker compose -f docker/docker-compose.yml up -d dev
fi

# Docker compose up builds and starts the container
echo "Attaching to container"
docker compose -f docker/docker-compose.yml exec dev bash
