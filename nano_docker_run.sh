#!/usr/bin/env bash

ARGS=("$@")

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<<"$xauth_list")
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
   else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi


#Prevent executing "docker run" when xauth failed.
if [ ! -f $XAUTH ]; then
    echo "[$XAUTH] was not properly created. Exiting..."
   exit 1
fi

docker run -it \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=$XAUTH \
    -v "$XAUTH:$XAUTH" \
    -v "/home/$USER/duckiepond-nctu:/home/argduckiepond/duckiepond-nctu/" \
    -v "/home/$USER/moos-ivp-taiwanMoos:/home/argduckiepond/moos-ivp-taiwanMoos/" \
    -v "/home/$USER/mr-bridge:/home/argduckiepond/mr_bridge/" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev:/dev" \
    -v "/home/$USER/.bashrc:/home/argduckiepond/.bashrc" \
    --workdir "/home/argduckiepond/" \
    --user root \
    --name argduckiepond \
    --runtime nvidia \
    --network host \
    --rm \
    --privileged \
    argnctu/duckiepond:arm64v8-nano \
    bash 