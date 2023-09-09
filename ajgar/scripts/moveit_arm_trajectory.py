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
        print ("============ Reference frame: %s" % planning_frame)
        
        eef_link = group.get_end_effector_link()
        print ("============ End effector: %s" % eef_link)

        group_names = robot.get_group_names()
        print ("============ Robot Groups:", robot.get_group_names())

        print ("============ Printing robot state")
        print (robot.get_current_state())
        print ("")

        self.box_name = ''
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
        
        pose_goal.position.x = 0.9
        pose_goal.position.y = 0.7
        pose_goal.position.z = 0.4
        

        pose_goal.orientation.w = 1.0
        
        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()



def main():
    interface = MoveGroupPythonInteface()
    interface.go_to_pose_goal()


if __name__ == '__main__':
    main()
    
#rosrun tf static_transform_publisher 0.9 0.7 0.4 0 0 0 1  world rnd_tf 100


