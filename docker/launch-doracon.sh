#!/bin/bash

# If you want to force rebuild, run the script with -f flag
if [[ "$1" == "-f" ]]; then
	docker compose -f docker/docker-compose.yml up dora --build
else
	docker compose -f docker/docker-compose.yml up dora
fi

# Launch container
docker exec dora -ti /bin/sh
