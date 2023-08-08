# POINTCLOUD SURFACE DETECTION
## generates downsampled pointclouds and normals


- [PCL] - Point Cloud Library
- [Rviz] - Visualizer for the simulation
- [Gazebo] - Open-source 3D robotics simulator
- [VScode] - Visual Studio Codes
- [Linux] - Ubuntu 20.04 is a must
- [Kinect] - azure kinect xbox 360 RGBD camera
- [C++] - C++ programming language


## Installation
Clone the important repositary, you may run the following command at your workspace directory.
```sh
git clone git@github.com:ros-drivers/freenect_stack.git
```
run the following commands starting at home directory.

```sh
git clone git@github.com:OpenKinect/libfreenect.git
cd libfreenect
mkdir build
cd build
cmake -L ..
make
sudo make install
sudo ldconfig /usr/local/lib64/
```

Install the dependency.

```sh
sudo apt install ros-noetic-rgbd-launch
```



