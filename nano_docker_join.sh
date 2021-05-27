#!/usr/bin/env bash
#
# Typical usage: ./join.bash subt
#

IMG=argnctu/duckiepond:arm64v8-xavier

xhost +
containerid=$(docker ps -aqf "ancestor=${IMG}")&& echo $containerid
docker exec --privileged -e DISPLAY=${DISPLAY} -e LINES="$(tput lines)" -it ${containerid} bash
xhost -