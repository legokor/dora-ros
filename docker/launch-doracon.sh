#!/bin/bash

# Check if container exists
if !(docker container list | grep -q "dora-ros-dora"); then
	echo "Docker container not found. Composing:"
	docker compose up dora
else
	echo "Container found, executing."
fi

# Launch container
docker exec dora -ti /bin/sh
