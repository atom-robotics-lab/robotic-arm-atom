cmake_minimum_required(VERSION 2.8.3)
project(plugin_collision_publish)
 
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
)
 
find_package(gazebo REQUIRED)
 
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
 
add_library(plugin_collision_publish src/plugin_collision_publish.cpp)
target_link_libraries(plugin_collision_publish ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
 
 
catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

