#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
import math
from std_msgs.msg import String

import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils

from prettytable import PrettyTable
from moveit_msgs.msg import MoveGroupActionResult



class ikSolverClass(object):
    
    def __init__(self):
        
        super(ikSolverClass, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous=True)
        robot         = moveit_commander.RobotCommander()
        group_name    = "arm_group"
        self.group    = moveit_commander.MoveGroupCommander(group_name)
        self.pi       = 22/7
        rospy.Subscriber("/move_group/result", MoveGroupActionResult, self.callback)
        
    def callback(self, data) :
        
        print("Trajectories : \n")
        
        jointtable = PrettyTable(["BJ", "SJ", "EJ", "LWJ", "UWJ"])

        trajectoriesLen = len(data.result.planned_trajectory.joint_trajectory.points)
        
        for value in range (0, trajectoriesLen) :
  
              joints = data.result.planned_trajectory.joint_trajectory.points[value].positions
              jointtable.add_row([str(round((joints[0] * 180/self.pi),1)),
                                  str(round((joints[1] * 180/self.pi),1)),
                                  str(round((joints[2] * 180/self.pi),1)),
                                  str(round((joints[3] * 180/self.pi),1)),
                                  str(round((joints[4] * 180/self.pi),1))])
              
        print(jointtable)

    
    def go_to_joint_state(self,angleList):
        
        print ("----------")
        
        joint = [ '                 ',
                  'Base Joint       ', 
                  'Shoulder Joint   ', 
                  'Elbow Joint      ', 
                  'Lower Wrist Joint', 
                  'Upper Wrist Joint', 
                  'Suction          ']
                  

        joint_goal = self.group.get_current_joint_values()
           
        for angle in range(1,6) :
        
            print(joint[angle] ," ", str(angleList[angle]* (180/self.pi)))	
        
        print ("----------") 	 
                 
        joint_goal[0] = angleList[1]
        joint_goal[1] = angleList[2]
        joint_goal[2] = angleList[3]
        joint_goal[3] = angleList[4]
        joint_goal[4] = angleList[5]
                
        self.group.go(joint_goal, wait=True)
        #self.group.stop()
        print("rosrun tf static_transform_publisher", self.target_position[0], self.target_position[1], self.target_position[2], self.roll_rad, self.pitch_rad, self.yaw_rad, " base_link rnd_tf 100")
        
    
    def ik_solver(self) :
        
        my_chain = ikpy.chain.Chain.from_urdf_file("arm.urdf")
        self.target_position = [0.2, -0.3, 0.3]

        roll_deg = 10.0
        pitch_deg = -92.0
        yaw_deg = -10.0
        self.roll_rad = math.radians(roll_deg)
        self.pitch_rad = math.radians(pitch_deg)
        self.yaw_rad = math.radians(yaw_deg)
        self.target_orientation = [[self.roll_rad, 0, 0],[0, self.pitch_rad, 0],[0, 0, self.yaw_rad]]
        
        self.go_to_joint_state(my_chain.inverse_kinematics(self.target_position, self.target_orientation, orientation_mode="all"))
         
	
ikSolver = ikSolverClass() 
ikSolver.ik_solver()