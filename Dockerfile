FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash","-c"]

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /workspaces/robo_arm_ws/devel/setup.bash" >> ~/.bashrc

RUN apt update

RUN apt install tmux libfreenect-dev -y

RUN apt install ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-rgbd-launch -y

RUN apt install ros-noetic-moveit -y

RUN mkdir -p /workspaces/robo_arm_ws/src

RUN rm -rf /var/lib/apt/lists/* \
&& apt-get clean

CMD ["sleep", "infinity"]
