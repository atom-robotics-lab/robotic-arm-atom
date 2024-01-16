FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash","-c"]

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /workspaces/robo_arm_ws/devel/setup.bash" >> ~/.bashrc

RUN apt-get update

RUN apt-get install apt-get install --no-install-recommends -yqqq \
    python3-pip 

RUN pip3 install ikpy prettytable ultralytics 

RUN apt-get install --no-install-recommends -yqqq \
    tmux \
    libfreenect-dev 

RUN apt-get install --no-install-recommends -yqqq \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-rgbd-launch \
    ros-noetic-moveit

RUN mkdir -p /workspaces/robo_arm_ws/src

RUN rm -rf /var/lib/apt/lists/* \
&& apt-get clean

CMD ["sleep", "infinity"]
