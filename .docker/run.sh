#!/bin/bash

# Command-line arguments:
# $1 - name of docker image variant (e.g. indigo, kinetic, kinetic-gpu, noetic)

# Following lines are based upon code from the ROS wiki
# See http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration#nvidia-docker2
# Copyright 2020 Open Source Robotics Foundation (CC-BY 3.0 license)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

docker run  -it \
            --env="DISPLAY=$DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            --env="XAUTHORITY=$XAUTH" \
            --volume="$XAUTH:$XAUTH" \
            --runtime=nvidia \
            spencer/spencer_people_tracking:$1 \
            tmux
