#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String


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
       

        self.robot = robot
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names



    def go_to_joint_state(self):
        group = self.group
    
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
      
    
        group.go(joint_goal, wait=True)
        group.stop()
    
    def go_to_pose_goal(self):

        group = self.group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.position.x = 0.3
        pose_goal.position.y = 0.2
        pose_goal.position.z = 0.3
        
        # group.setTargetPose(0.3, 0.2, 0.5, group.getEndEffectorLink())
        pose_goal.orientation.w = 0
        
        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()



def main():
    interface = MoveGroupPythonInteface()
    interface.go_to_pose_goal()


if __name__ == '__main__':
    main()