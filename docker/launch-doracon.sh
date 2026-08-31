#!/bin/bash

# Run from project root directory with "docker/launch-doracon.sh" !

# If you want to force rebuild, run the script with -f flag
if [[ "$1" == "-f" ]]; then
	docker compose -f docker/docker-compose.yml up -d --build dora
else
	docker compose -f docker/docker-compose.yml up -d dora
fi

# Docker compose up builds and starts the container
echo "Attaching to container"
docker compose -f docker/docker-compose.yml exec dora bash
