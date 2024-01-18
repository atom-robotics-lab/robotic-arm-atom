cmake_minimum_required(VERSION 3.0.2)
project(ajgar_core)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
)

 add_service_files(
   FILES
   tfValueSrv.srv
 )

 generate_messages(
   DEPENDENCIES
    geometry_msgs   
    std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

