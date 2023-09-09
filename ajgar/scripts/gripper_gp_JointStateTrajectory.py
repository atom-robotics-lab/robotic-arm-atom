#!/usr/bin/env python3

import rospy
import actionlib
import control_msgs.msg

class FollowJointTrajectoryAction(object):
    _feedback = control_msgs.msg.FollowJointTrajectoryActionFeedback()
    _result = control_msgs.msg.FollowJointTrajectoryActionResult()
    
    print ("Controller Gripper Group Loaded")

    def __init__(self):
        self._action_name = 'hand_ee_controller/follow_joint_trajectory'
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
    def execute_cb(self, goal):
        r = rospy.Rate(1)
        print(goal)
        self._as.set_succeeded()
       

if __name__ == '__main__':
    rospy.init_node('controller_gripper_gp')
    server = FollowJointTrajectoryAction()
    rospy.spin()