#!/usr/bin/env python3

# standard library imports
import sys
import os
import rospy
import math

# ROS specific imports
import rospkg
import moveit_commander

# custom imports
# Get the path to the package using `rospkg` and add the path to the `sys.path`
# This is done to import the custom python scripts from the package
rospack = rospkg.RosPack()
pkg_file_path = rospack.get_path("plugin_pneumatic_gripper")
script_import_path = os.path.join(str(pkg_file_path), "scripts")
sys.path.insert(1, script_import_path)
import attach
import detach

import ikpy.chain
from prettytable import PrettyTable
from std_msgs.msg import String

from ajgar_core.srv import tfValueSrv, tfValueSrvResponse


class ikSolverClass(object):
    
    def __init__(self):
        
        # ROS node and service 
        node = 'move_group_python_interface'
        srvnode = 'ikSolverSrv'
        group_name  = "arm_group"
        self.moveit_result = "/move_group/result"
        self.gazebo_collision = "/gazebo/collision/info"        
        
        # ROS node initialization 
        rospy.init_node(node, anonymous=True)
        
        # MoveIt! commander initialization
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander(group_name)
       
       # global variables initialization
        self.obj2 = None
        self.count = 0
        
        rospy.Service(srvnode, tfValueSrv, self.tfValueSrvCallback)
        print(" ikpy/Moveit! :  Ready with ROS Server ")


    def tfValueSrvCallback(self, msg):
        
        print("------- ")
        print(" ikpy/Moveit! : Events log " )

        collisionBool = False

        rospack = rospkg.RosPack()
        package_path = rospack.get_path("ajgar_description")
        modelPath = package_path + "/urdf/ajgar.xacro"

        # ------ Function for Pick up box ------ #
        
        print(" - tf rcvd, solving for joints angle, calling ikpy "  )
        tfValue = [msg.tfArray[0], msg.tfArray[1], msg.tfArray[2]]
        anglesState = self.ik_solver(tfValue, modelPath)

        # xxxxx---------------xxxxxxxxx #
        # TODO : Orientation value for pick 
        # yaw_deg = 0.10
        # roll_deg = 0.10
        # pitch_deg = -0.90 
        # self.yaw_rad = math.radians(yaw_deg)
        # self.roll_rad = math.radians(roll_deg)
        # self.pitch_rad = math.radians(pitch_deg)
        #self.target_orientation = [[self.roll_rad, 0, 0],[0, self.pitch_rad, 0],[0, 0, self.yaw_rad]]
        # xxxxx---------------xxxxxxxxx #
        
        
        print(" - angle rcvd, solving for trajectory, calling Moveit! "  )
        self.moveitGoToJointState(anglesState) 
        
        # xxxxx---------------xxxxxxxxx #
        # TODO : implement when trajectory needed  
        # print(" Listening @ /move_group/result ")
        # trajectory = rospy.wait_for_message(self.moveit_result, String)
        # self.trajectory(trajectory)
        # xxxxx---------------xxxxxxxxx #
        
        print(" - arm reached, Detecting collision ")
        print(" - listening @ /gazebo/collision/info ")
        while ( not collisionBool ) : 
             collisionData = rospy.wait_for_message(self.gazebo_collision, String)
             collisionBool, boxId = self.collision_detection(collisionData)
        
        print(" Object detected ")
        
        attach.attach_links(boxId)
        
        print(" Object attached ")

        # # ------ Function for Place box ------ #

        # xxxxx---------------xxxxxxxxx #
        # TODO : Orientation value for place 
        # roll_deg = 0.10
        # pitch_deg = -0.90
        # yaw_deg = 0.10
        # self.roll_rad = math.radians(roll_deg)
        # self.pitch_rad = math.radians(pitch_deg)
        # self.yaw_rad = math.radians(yaw_deg)
        #self.target_orientation = [[self.roll_rad, 0, 0],[0, self.pitch_rad, 0],[0, 0, self.yaw_rad]]
        # xxxxx---------------xxxxxxxxx #
        
        tfValue = [-0.24 , -0.2 , 0.05]
        
        print(" - dropping zone tf rcvd, solving for joints angle, calling ikpy ")
        anglesState = self.ik_solver(tfValue, modelPath)
        
        
        print(" - angle rcvd, solving for trajectory, calling Moveit! "  )
        self.moveitGoToJointState(anglesState) 
        
        # print(" Listening @ /move_group/result ")
        # trajectory = rospy.wait_for_message(self.moveit_result, String)
        # self.trajectory(trajectory)
        
        detach.detach_links(boxId)
        print(" Object detached ")
        
        return tfValueSrvResponse(True)
        
        
    def trajectory(self, data) :
        print("Trajectories : \n")        
        jointtable = PrettyTable(["BJ", "SJ", "EJ", "LWJ", "UWJ"])
        trajectoriesLen = len(data.result.planned_trajectory.joint_trajectory.points)

        for value in range (0, trajectoriesLen) :  
              joints = data.result.planned_trajectory.joint_trajectory.points[value].positions
              jointtable.add_row([str(round((joints[0] * 180/math.pi),1)),
                                  str(round((joints[1] * 180/math.pi),1)),
                                  str(round((joints[2] * 180/math.pi),1)),
                                  str(round((joints[3] * 180/math.pi),1)),
                                  str(round((joints[4] * 180/math.pi),1))])
              
        print(jointtable)

    
    def moveitGoToJointState(self,angleList):
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
            print(joint[angle] ," ", str(angleList[angle]* (180/math.pi)))	
        
        print ("----------") 	                 
        joint_goal[0] = angleList[1]
        joint_goal[1] = angleList[2]
        joint_goal[2] = angleList[3]
        joint_goal[3] = angleList[4]
        joint_goal[4] = angleList[5]
                
        self.group.go(joint_goal, wait=True)
        #self.group.stop()
        
    
    def collision_detection(self, data):
        objects = str(data).split(' ')[1].split('-')
        collision_object_1, collision_object_2 = objects[0][1:], objects[1][:-1]
        if "ajgar::end::end_collision" == collision_object_1:
            obj2 = collision_object_2
            return True, obj2 
        return False, None
            

    def ik_solver(self, msg, modelPath) :
        my_chain = ikpy.chain.Chain.from_urdf_file(modelPath)
        X , Y , Z = msg[0], msg[1], msg[2]
        target_position = [X, Y, Z]        
        return my_chain.inverse_kinematics(target_position)
        

	
if __name__ == "__main__":
    while not rospy.is_shutdown():
        ikSolver = ikSolverClass() 
        rospy.spin()
