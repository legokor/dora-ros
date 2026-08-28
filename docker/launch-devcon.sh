#!/bin/bash

# Run from root directory with "docker/launch-devcon.sh" !

# If you want to force rebuild, run the script with -f flag
if [[ "$1" == "-f" ]]; then
	docker compose -f docker/docker-compose.yml up dev --build
else
	docker compose -f docker/docker-compose.yml up dev
fi

# Enabling X11 forwarding support. 
xhost +local:docker
docker exec -i -t dev /bin/sh


