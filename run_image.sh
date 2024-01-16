#!/bin/bash

# Map host's display socket to docker
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")
# DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")

xhost +local:root

container_name="ajgar-docker-test"
image_name="moveit-classic-noetic"

if docker ps --format '{{.Names}}' | grep -q "$container_name"; then
    docker exec -it $container_name /bin/bash

else
    docker run -it --rm \
        ${DOCKER_ARGS[@]} \
        -e DISPLAY=$DISPLAY \
        -v $PWD/build_files:/workspaces/robo_arm_ws/ \
        -v $PWD:/workspaces/sim_ws/src \
        -v /etc/localtime:/etc/localtime:ro \
        --name "$container_name" \
        --runtime nvidia \
        --workdir /workspaces/robo_arm_ws \
        $@ \
        "$image_name":1.0 \
        /bin/bash
fi
