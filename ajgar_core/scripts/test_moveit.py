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
        group_name = "arm_group"
        group = moveit_commander.MoveGroupCommander(group_name)
        

       # planning_frame = group.get_planning_frame()
       # print ("============ Reference frame: %s" % planning_frame)
        
       # eef_link = group.get_end_effector_link()
       # print ("============ End effector: %s" % eef_link)

        #group_names = robot.get_group_names()
        #print ("============ Robot Groups:", robot.get_group_names())

        #print ("============ Printing robot state")
        #print (robot.get_current_state())
        #print ("")

        #self.box_name = ''
        #self.robot = robot
        self.group = group
        #self.display_trajectory_publisher = display_trajectory_publisher
        #self.planning_frame = planning_frame
        #self.eef_link = eef_link
        #self.group_names = group_names



    def go_to_joint_state(self):

        group = self.group
                
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = -1
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        # joint_goal[4] = 0
      
    
        group.go(joint_goal, wait=True)
        group.stop()
    
    def go_to_pose_goal(self):

        arm_group = moveit_commander.MoveGroupCommander("arm_group")
        gripper_group = moveit_commander.MoveGroupCommander("gripper_group")
        
        print(arm_group.get_current_pose())
        print(gripper_group.get_current_pose())
        
        unit_range = 0.1
        arm_target_pose = geometry_msgs.msg.Pose()
        
        
        arm_target_pose.orientation.w = 1
        arm_target_pose.position.x = 0.300000007
        arm_target_pose.position.y = 0.500000005 
        arm_target_pose.position.z = 0.409988800 
        #arm_group.set_num_planning_attempts(20)
        #arm_group.set_goal_tolerance(00.01)
        #arm_target_pose.orientation.x = 0
        #arm_target_pose.orientation.y = 0
        #arm_target_pose.orientation.z = 0

	
        
        arm_group.set_pose_target(arm_target_pose)
        plan = arm_group.plan()
        print(plan) 
        
        #success = arm_group.go(wait=True)
        #arm_group.stop()
        #arm_group.clear_pose_targets()
      
    # Plan and execute the arm and gripper motion
        #arm_group.go()
        # gripper_group.go()



def main():
    interface = MoveGroupPythonInteface()
    interface.go_to_pose_goal()


if __name__ == '__main__':
    main()
    
#rosrun tf static_transform_publisher -6.84879e-07 -0.23105 1.48459 8.47014e-12 -1.37682e-06 5.96021e-06 1  base_link rnd_tf 100

# rosrun tf static_transform_publisher 0.3 0.2 0.3 0 1 0 world rnd_tf 100

