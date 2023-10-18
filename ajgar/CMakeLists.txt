cmake_minimum_required(VERSION 2.8.3)

project(ajgar)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	rospy
	std_msgs
	geometry_msgs
	urdf
	xacro
	message_generation
)

add_service_files(
FILES
bool_service.srv
)

generate_messages(
DEPENDENCIES
std_msgs
)

catkin_package(CATKIN_DEPENDS
	geometry_msgs
	roscpp
	rospy
	std_msgs
)

find_package(roslaunch)

foreach(dir config launch meshes urdf )
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
