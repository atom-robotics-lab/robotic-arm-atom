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


add_executable(ply_extractor src/ply_extractor.cpp)
target_link_libraries(ply_extractor 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(boundary_detector src/boundary.cpp)
target_link_libraries(boundary_detector 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(filter src/filter.cpp)
target_link_libraries(filter 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(octomapFilter src/octomapFilter.cpp)
target_link_libraries(octomapFilter 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(pcl_visualizer_demo src/pcl_visualizer_demo.cpp)
target_link_libraries(pcl_visualizer_demo 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
