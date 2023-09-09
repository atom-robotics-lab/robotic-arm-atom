#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def JointStatePublisher():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')
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
    
    JointState_str.position = [0, 0, 0, 0, 0, 0]
    JointState_str.velocity = []
    JointState_str.effort = []

    while not rospy.is_shutdown():
      JointState_str.header.stamp = rospy.Time.now()
      pub.publish(JointState_str)
      rate.sleep()

if __name__ == '__main__':
    try:
        JointStatePublisher()
    except rospy.ROSInterruptException:
        pass
