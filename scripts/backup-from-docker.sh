#!/bin/bash

F=~/backup/dora-ros/$(date +%FT%T).tar.gz ; docker exec -it great_elgamal bash -c $'tar -cxOC ~ $( printf -- \'--exclude=./dora-ros/%s\n\' ros2_ws/src/{build,log,install,rplidar_ros} .git) dora-ros | base64' | tr -d '\n\r' | base64 -d > $F ; file $F
