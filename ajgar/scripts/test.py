#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String
 

def JointStatePublisher(data):

    rate = rospy.Rate(10) 
    
   
    JointState_str = JointState()
    JointState_str.header = Header()
    JointState_str.header.stamp = rospy.Time.now()
    JointState_str.name = ['base_joint',
                           'shoulder_joint',
                           'arm_joint',
                           'forearm_joint',
                           'wrist_joint',
                           'suction_joint']
    
    JointState_str.position = [float(data.data), 0, 0, 0, 0, 0]
    JointState_str.velocity = []
    JointState_str.effort = []
    JointState_str.header.stamp = rospy.Time.now()
    pub.publish(JointState_str)
      

def ArduinoMoveitInterface() :
	rospy.Subscriber("arduino_controller", String, JointStatePublisher)
	rospy.spin()

if __name__ == '__main__':
    rospy.init_node('joint_state_publisher')
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    ArduinoMoveitInterface()
