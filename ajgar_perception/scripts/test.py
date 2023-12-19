#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from ajgar_perception.srv import AddTwoInts

class PointCloudModifier:

    def __init__(self):
       
        rospy.init_node('point_cloud_modifier')
        
        self.kinectPCTopic =  "/kinect/depth/image_raw" 

        self.kinectPCData = None
        self.maskPCData   = None
        self.pointBool    = None 

        self.cnt = 0 

        
        
    def main(self):
        print(" Listening @ /kinect/depth/image_raw ")
        self.kinectPCData = rospy.wait_for_message(self.kinectPCTopic, PointCloud2) 
        print(" data rcvd ")
        
        # print(" Listening @ /mask")
        # self.maskPCData   = rospy.wait_for_message(self.maskPCtopic, PointCloud2)
        # print(" data rcvd ")
        

if __name__ == "__main__":

    modifier = PointCloudModifier()
    modifier.main()
