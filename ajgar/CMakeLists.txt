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
	actionlib
	genmsg
	actionlib_msgs
	trajectory_msgs
	control_msgs
	)

add_action_files(DIRECTORY action FILES JointStateController.action)
generate_messages(DEPENDENCIES actionlib_msgs trajectory_msgs control_msgs)

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
