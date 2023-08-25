cmake_minimum_required(VERSION 2.8.3)
project(plugin_collision_publish)
 
## Add support for C++11, supported in ROS Kinetic and newer
 
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)
 
# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
 
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
 
# For tc_simple_world_plugin plugin
add_library(plugin_collision_publish src/plugin_collision_publish.cpp)
target_link_libraries(plugin_collision_publish ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
 

 
catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

