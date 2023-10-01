#! /usr/bin/env python3



touch Master_Data.srv


import rospy
from your_package.srv import GetData, GetDataResponse
from sensor_msgs.msg import YourMessageType


rospy.init_node("MASTER_NODE")