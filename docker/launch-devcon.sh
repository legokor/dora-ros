#!/bin/bash

# To launch windows version, execute the file with the "windows" argument

# Check if container exists
if docker container list -a | grep -q "docker-dev"; then
	echo "Container found, executing."
else
	echo "Docker container not found. Composing:"
	docker compose up dev
fi

if [[ "$1" == "windows" ]]; then
    echo "ok"
	# Launch container
	docker exec dev -ti /bin/sh
else
    echo "ok"
	# Enabling X11 forwarding support. 
	xhost +local:docker
	# Launching with X11 in mind, mounting socket
	docker exec dev -ti /bin/sh --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix
fi


