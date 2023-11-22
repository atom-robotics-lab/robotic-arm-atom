#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import math
from std_msgs.msg import String, Int32
import tf
import tf2_msgs.msg
import tf2_ros
import ikpy.chain
from prettytable import PrettyTable
from moveit_msgs.msg import MoveGroupActionResult
from plugin_pneumatic_gripper.srv import Attach, AttachRequest, AttachResponse

import attach
import detach

class ikSolverClass(object):
    
    def __init__(self):
        
        super(ikSolverClass, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous=True)
        robot         = moveit_commander.RobotCommander()
        group_name    = "arm_group"
        self.group    = moveit_commander.MoveGroupCommander(group_name)
        self.pi       = 22/7

        self.count = 0
        self.obj2 = None

        self.X = None
        self.Y = None
        self.Z = None

        self.bool_publish = rospy.Publisher("imgProcessBool", Int32)
        self.bool_publish.publish(1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, self.tf_callback, queue_size=1)
        rospy.Subscriber("/move_group/result", MoveGroupActionResult, self.callback)
        rospy.Subscriber("/gazebo/collision/info", String, self.collision_callback, queue_size=1)
        
    def tf_callback(self, data):

        for transform_stamped in data.transforms:
            transform = transform_stamped.transform
            
            if transform_stamped.header.frame_id == 'base_link' and transform_stamped.child_frame_id == 'box' : 
                print(f"  Frame ID: {transform_stamped.header.frame_id}")
                print(f"  Child Frame ID: {transform_stamped.child_frame_id}")
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
                
                rospy.sleep(5)
        
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

    def place(self):
        
        my_chain = ikpy.chain.Chain.from_urdf_file("arm.urdf")

        self.target_position = [-0.24 , -0.2 , 0.05]
        roll_deg = 0.10
        pitch_deg = -0.90
        yaw_deg = 0.10
        self.roll_rad = math.radians(roll_deg)
        self.pitch_rad = math.radians(pitch_deg)
        self.yaw_rad = math.radians(yaw_deg)
        self.target_orientation = [[self.roll_rad, 0, 0],[0, self.pitch_rad, 0],[0, 0, self.yaw_rad]]
        
        self.go_to_joint_state(my_chain.inverse_kinematics(self.target_position)) #, self.target_orientation, orientation_mode="all"))
        detach.detach_links(self.obj)
        self.obj2 = None
        self.obj = None
        self.X = None
        self.Y = None
        self.Z = None
        self.bool_publish.publish(1)
        rospy.sleep(0.1)
        self.ik_solver()

    def collision_callback(self, data):

        objects = str(data).split(' ')[1].split('-')
        collision_object_1, collision_object_2 = objects[0][1:], objects[1][:-1]

        #print(f"{collision_object_1} and {collision_object_2} \n")
        if "ajgar::suction::suction_collision" == collision_object_1:
            self.count += 1
            self.obj2 = collision_object_2
            # print(f"\nHit: {self.obj2} \nCount: {self.count}")                

    def ik_solver(self) :
        
        my_chain = ikpy.chain.Chain.from_urdf_file("arm.urdf")
        self.bool_publish.publish(0)
        while True:
            if self.X != None:    
                break

        self.target_position = [self.X, self.Y, self.Z]
        # roll_deg = self.roll_deg 
        # pitch_deg = self.pitch_deg 
        # yaw_deg = self.yaw_deg

        # self.target_position = [0.16, -0.072, 0.100] 
        roll_deg = 0.10
        pitch_deg = -0.90
        yaw_deg = 0.10
        self.roll_rad = math.radians(roll_deg)
        self.pitch_rad = math.radians(pitch_deg)
        self.yaw_rad = math.radians(yaw_deg)
        self.target_orientation = [[self.roll_rad, 0, 0],[0, self.pitch_rad, 0],[0, 0, self.yaw_rad]]
        
        self.go_to_joint_state(my_chain.inverse_kinematics(self.target_position)) #, self.target_orientation, orientation_mode="all"))
        while True:
            if self.obj2 != None:
                rospy.logwarn("inside loop")
                self.obj = self.obj2
                attach.attach_links(self.obj)
                break
        self.place()

	
if __name__ == "__main__":
    while not rospy.is_shutdown():

        ikSolver = ikSolverClass() 
        ikSolver.ik_solver()