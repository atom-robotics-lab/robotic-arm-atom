#!/bin/bash

image_name="ajgar-docker:latest"
container_name="ajgar"

if docker images --format '{{.Repository}}' | grep -q "$image_name"; then
    echo "Updating the existing Docker image: $image_name:1.0"

    docker run -it --rm=false \
        --privileged \
        --network host \
        ${DOCKER_ARGS[@]} \
        -e DISPLAY=$DISPLAY \
        -v $PWD:/workspaces/robo_arm_ws/src \
        -v /etc/localtime:/etc/localtime:ro \
        --name "$container_name" \
        --runtime nvidia \
        --workdir /workspaces/robo_arm_ws \
        $@ \
        "$image_name" \
        bash -c "source /opt/ros/iron/setup.bash && colcon build --symlink-install && source install/setup.bash && exit"

    docker commit "$container_name" "$image_name"

    docker rm "$container_name"

else
    echo "Building a new Docker image: $image_name"
    docker build -f Dockerfile -t "$image_name" .
fi