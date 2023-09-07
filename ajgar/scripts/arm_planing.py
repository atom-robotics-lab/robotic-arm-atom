#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
class MoveGroupPythonInteface(object):

    def __init__(self):

        super(MoveGroupPythonInteface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous=True)

        robot = moveit_commander.RobotCommander()

        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        planning_frame = group.get_planning_frame()
        eef_link = group.get_end_effector_link()
        group_names = robot.get_group_names()

        # Initialize a subscriber for the desired pose
        self.pose_subscriber = rospy.Subscriber('/pose', PoseStamped, self.pose_callback)

        self.robot = robot
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.desired_pose = None  # Initialize the desired_pose attribute

    # Callback function to handle incoming pose messages
    def pose_callback(self, pose_msg):

        # Extracting position (x, y, z)
        self.desired_pose = pose_msg.pose

        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y
        z = pose_msg.pose.position.z

        # Extracting orientation (w)
        w = pose_msg.pose.orientation.w
        rospy.loginfo(f"Received Pose: x={x}, y={y}, z={z}, w={w}")

        self.go_to_pose_goal()

    def go_to_pose_goal(self):
        group = self.group

        # Check if a desired pose has been received
        if self.desired_pose is None:

            rospy.logwarn("No desired pose received yet.")
            return
               
        
        # Use the received pose as the target
        pose_goal = self.desired_pose
        rospy.loginfo(pose_goal)


        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

def main():
    interface = MoveGroupPythonInteface()
    interface.go_to_pose_goal()
    
    # Spin the ROS node to handle incoming messages
    rospy.spin()

if __name__ == '__main__':
    main()