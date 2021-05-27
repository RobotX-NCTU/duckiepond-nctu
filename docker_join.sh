#!/usr/bin/env bash
#
# Typical usage: ./join.bash duckiepond
#

IMG=argnctu/duckiepond:ipc

containerid=$(docker ps -aqf "ancestor=${IMG}") && echo $containerid
docker exec -it \
    --privileged \
    -e DISPLAY=${DISPLAY} \
    -e LINES="$(tput lines)" \
    ${containerid} \
    bash