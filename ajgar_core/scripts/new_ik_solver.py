#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import math
import tf
import tf2_msgs.msg
import tf2_ros
import ikpy.chain
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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, self.tf_callback)
        rospy.Subscriber("/move_group/result", MoveGroupActionResult, self.callback)


    def tf_callback(self, data):
        print("Received TF messages:")
        # if self.tf_buffer.can_transform('base_link','camera_depth_optical_frame', rospy.Time(0)):
        #     trans = self.tf_buffer.lookup_transform('base_link', 'camera_depth_optical_frame', rospy.Time(0))
        #     print("+++++++++++++")
        #     print("transformed values are: ")
        #     print(trans)
        #     print("++++++++++++++++++")

        # else:
        #     print("+++++++++++++++++++++++++")
        #     rospy.logdebug("no transform possible")
        #     print("+++++++++++++++++++++++++")

        for transform_stamped in data.transforms:
            transform = transform_stamped.transform
            print(f"  Frame ID: {transform_stamped.header.frame_id}")
            print(f"  Child Frame ID: {transform_stamped.child_frame_id}")
            if transform_stamped.header.frame_id == 'base_link' and transform_stamped.child_frame_id == 'box' : 
                print("  Translation:")
                print(f"    x: {transform.translation.x}")
                print(f"    y: {transform.translation.y}")
                print(f"    z: {transform.translation.z}")
                # print("  Rotation:")
                # print(f"    x: {transform.rotation.x}")
                # print(f"    y: {transform.rotation.y}")
                # print(f"    z: {transform.rotation.z}")
                # print(f"    w: {transform.rotation.w}")
                # print("----------")

                self.X = transform.translation.x
                self.Y = transform.translation.y
                self.Z = transform.translation.z

        # Convert quaternion to degrees directly within the callback
                quaternion = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
                magnitude = math.sqrt(sum(x ** 2 for x in quaternion))
                quaternion_normalized = [x / magnitude for x in quaternion]

                roll = math.atan2(2 * (quaternion_normalized[1]*quaternion_normalized[3] - quaternion_normalized[0]*quaternion_normalized[2]), 1 - 2*(quaternion_normalized[1]**2 + quaternion_normalized[2]**2))
                pitch = math.asin(2 * (quaternion_normalized[0]*quaternion_normalized[1] + quaternion_normalized[2]*quaternion_normalized[3]))
                yaw = math.atan2(2 * (quaternion_normalized[0]*quaternion_normalized[3] - quaternion_normalized[1]*quaternion_normalized[2]), 1 - 2*(quaternion_normalized[0]**2 + quaternion_normalized[1]**2))

                self.roll_deg = math.degrees(roll)
                self.pitch_deg = math.degrees(pitch)
                self.yaw_deg = math.degrees(yaw)

                print(f"  Euler Angles (degrees): Roll: {self.roll_deg}, Pitch: {self.pitch_deg}, Yaw: {self.yaw_deg}")
                print("----------")
        
        rospy.sleep(2)
        
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

        self.target_position = [self.X, self.Y, self.Z - 0.05]
        # roll_deg = self.roll_deg 
        # pitch_deg = self.pitch_deg 
        # yaw_deg = self.yaw_deg

        # self.target_position = [0.45, 0.05, 0.16] 
        roll_deg = 0.10
        pitch_deg = -0.90
        yaw_deg = -0.10
        self.roll_rad = math.radians(roll_deg)
        self.pitch_rad = math.radians(pitch_deg)
        self.yaw_rad = math.radians(yaw_deg)
        self.target_orientation = [[self.roll_rad, 0, 0],[0, self.pitch_rad, 0],[0, 0, self.yaw_rad]]
        
        self.go_to_joint_state(my_chain.inverse_kinematics(self.target_position, self.target_orientation, orientation_mode="all"))
         
	
ikSolver = ikSolverClass() 
ikSolver.ik_solver()