#!/usr/bin/env python

import rospy
import moveit_commander
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
import sys
             

# Joint Angle Limits
dicAngle = { "base_joint" : [-3.14, 3.14], "shoulder_joint" : [-1.57 , 1.57], "arm_joint" : [-2.87, 2.87], "forearm_joint" : [-2.87, 2.87], "wrist_joint" : [-2.87, 2.87] }

def transform_callback(transform_stamped):
    print("this is the tf callback")
    try:
        print("try")
        # Convert the TransformStamped message to a Pose message
        target_pose = tf2_geometry_msgs.PoseStamped()
        target_pose.header = transform_stamped.header
        target_pose.pose.position = transform_stamped.transform.translation
        target_pose.pose.orientation = transform_stamped.transform.rotation

        # Set the target pose as the goal
        arm.set_pose_target(target_pose.pose)

        # Plan the motion
        plan = arm.plan()

        # Execute the planned motion
        arm.execute(plan)
    except Exception as e:
        print("error")
        rospy.logerr("Error processing transform: %s", str(e))

def move_to_target_pose():
    # Initializing
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm_to_target_pose', anonymous=True)

    # Create a RobotCommander instance to interface with your robot's planning groups and kinematics
    robot = moveit_commander.RobotCommander()


    # Create a global MoveGroupCommander for the arm
    global arm
    arm = moveit_commander.MoveGroupCommander("arm")

    # ROS parameters
    planning_frame = arm.get_planning_frame()
    eef_link = arm.get_end_effector_link()
    list_joint_values = arm.get_current_joint_values()

    #TF listener and subscriber
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf = rospy.Subscriber("/tf_box", geometry_msgs.msg.TransformStamped, transform_callback)

    # Print the ROS Parameters
    rospy.loginfo( "Planning Group: "    + "\033[94m" + "{}".format(planning_frame) + "\033[0m")
    rospy.loginfo( "End Effector Link: " + "\033[94m" + "{}".format(eef_link) + "\033[0m")
    rospy.loginfo("\033[94m" + "arm init done." + "\033[0m")
    rospy.loginfo("\033[94m" + ">>> Current Joint Values:" + "\033[0m")
    rospy.loginfo(list_joint_values)

    transform_callback(tf)

    rospy.spin()

    # Clean up
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_to_target_pose()
    except rospy.ROSInterruptException:
        pass
