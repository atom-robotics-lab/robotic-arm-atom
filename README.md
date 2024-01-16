# FlipkartGrid

This project involves the creation of a 6 Degrees of Freedom (DOF) robotic arm capable of picking and placing objects as desired.


### Package Descriptions


| Package | Description |
| --- | --- |
| `ajgar_core` | This package contains the core functionalities of the robotic arm, including the main control algorithms and launch files. |
| `ajgar_depend_pkgs` | This package includes the dependencies required for the freenect_stack (alternative for Kinect camera plugin). |
| `ajgar_description` | This package contains the URDF (Unified Robot Description Format) files for the robotic arm. These files describe the robot's physical configuration. |
| `ajgar_hardware` | This package is responsible for interfacing with the physical hardware of the robotic arm. It includes drivers and communication protocols. |
| `ajgar_moveit_config` | This package contains the configuration files for MoveIt, a ROS-based software for motion planning, kinematics, and robot interaction. |
| `ajgar_perception` | This package is responsible for the perception tasks, such as object recognition and environment mapping. |
| `ajgar_sim` | This package contains the simulation environment for the robotic arm. It includes models and simulation worlds. |
| `ajgar_sim_plugins` | This package contains plugins for the simulation environment, currently providing suction functionality. |
| `ur5_description` | This package contains the URDF files for a specific model of the robotic arm, the UR5. |
| `ur5_moveit_config` | This package contains the MoveIt configuration files for the UR5 robotic arm. |

## Installations 

### ROS Controller Installation

You need to install the ros controller to control the Arm using the moveit. To install it, run the following command

```sh
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
```

### Freenect Dependencies Installation 

You need to install the following freenect dependencies in order to use kinect 

```sh
sudo apt install libfreenect-dev
sudo apt-get install ros-noetic-rgbd-launch
```

## Run using Docker

1. Install Docker from [here](https://docs.docker.com/engine/install/ubuntu/)

2. Execute the following command to run Docker without using `sudo`

    ```sh
     sudo usermod -aG docker $USER
    ```

3. The command below will build the Docker image 

    ```sh
    ./build_image.sh
    ```

4. Run the image using this command

    ```sh
    ./run_image.sh
    ```

## Simulation  

### Launch Robotic ARM in Gazebo, RViz and MoveGroup 

1. To launch AJGAR model in RViz , Gazebo and MoveGroup :

    ```sh 
    roslaunch ajgar_core  ajgar_moveit.launch 
    ```

### Start all the services

2. To initialize and start all the services : 

    ```sh 
    roslaunch ajgar_core ajgar_rosservice.launch
    ```

### Start the pick-and-place procedure

3. To start the pick-and-place procedure : 

    ```sh 
    rosrun ajgar_perception server.py
    ```



