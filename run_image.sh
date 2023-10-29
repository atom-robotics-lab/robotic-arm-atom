#!/bin/bash

# Map host's display socket to docker
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")
# DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
DOCKER_ARGS+=("-e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml")

xhost +local:root

docker run -it --rm \
    --privileged \
    --network host \
    ${DOCKER_ARGS[@]} \
    -e DISPLAY=$DISPLAY \
    -v $PWD:/workspaces/robo_arm_ws/src \
    -v /etc/localtime:/etc/localtime:ro \
    --name "nvidia-sim-docker" \
    --runtime nvidia \
    --workdir /workspaces/robo_arm_ws \
    $@ \
    ajgar-docker:latest \
    /bin/bash

