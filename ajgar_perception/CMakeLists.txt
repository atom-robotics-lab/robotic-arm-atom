cmake_minimum_required(VERSION 3.0.2)
project(ajgar_perception)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  sensor_msgs
  message_generation
)

find_package(PCL REQUIRED)

 add_service_files(
   FILES
   octomapSrv.srv
   percepSrv.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(octomapFilter src/octomapFilter.cpp)
target_link_libraries(octomapFilter 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


