# FlipkartGrid

Creating a 5 DOF robotic arm to pick and place objects in a dersired manner.

### installation for ros controller

You need to install the ros controller to control the Arm using the moveit. To install it, run the following command

```sh
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
```

### installation for freenect dependencies

You need to install the following freenect dependencies in order to use kinect 

```sh
sudo apt install libfreenect-dev
sudo apt-get install ros-noetic-rgbd-launch
```

### run Gazebo and RViz 

To launch RViz and Gazebo 

```sh 
roslaunch ur5_moveit_config ajgar_sim.launch 
```

