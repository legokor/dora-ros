#!/bin/bash

# Run from root directory with "docker/launch-devcon.sh" !

# Enabling X11 forwarding support. 
xhost +local:docker

## Flags
# -f : force rebuild container

rebuild=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    -f) rebuild=true;;
    *) echo "Unknown flag: $1" >&2 ;;
  esac
  shift
done

# Compose up with -d flag to run in background
if $rebuild; then
  docker-compose -f docker/docker-compose.yml up --build -d dev
else
  docker-compose -f docker/docker-compose.yml up -d dev
fi

# Docker compose up builds and starts the container
echo "Attaching to container"
docker compose -f docker/docker-compose.yml exec dev bash
