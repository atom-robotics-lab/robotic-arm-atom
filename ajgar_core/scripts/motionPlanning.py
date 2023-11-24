#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
import rospkg
from math import pi
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
        
        rospack = rospkg.RosPack()
        self.package_path = rospack.get_path('ajgar_core') + '/scripts/arm.urdf'
        
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
        
                
        self.group.go(joint_goal, wait=True)
        #self.group.stop()

        
    
    def ik_solver(self) :
        
        my_chain = ikpy.chain.Chain.from_urdf_file(self.package_path)
        target_position = [0.37, -0.117, 0.165]
        pi = 22/7 
        self.go_to_joint_state(my_chain.inverse_kinematics(target_position))
         
	
ikSolver = ikSolverClass() 
ikSolver.ik_solver()
