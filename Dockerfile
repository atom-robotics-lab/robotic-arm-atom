FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash","-c"]

RUN apt update

RUN apt install tmux libfreenect-dev -y

RUN apt install ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-rgbd-launch -y

RUN source /opt/ros/humble/setup.bash && cd /workspaces/sim_ws/ && colcon build --symlink-install
RUN echo "source /workspaces/sim_ws/install/setup.bash" >> ~/.bashrc

RUN mkdir -p /workspaces/robo_arm_ws/src

RUN rm -rf /var/lib/apt/lists/* \
&& apt-get clean

CMD ["sleep", "infinity"]
