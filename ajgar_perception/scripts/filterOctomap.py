#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from ajgar_perception.srv import AddTwoInts

class PointCloudModifier:

    def __init__(self):
       
        rospy.init_node('point_cloud_modifier')
        
        self.kinectPCTopic =  "/kinect/depth/points" 
        self.maskPCtopic   =  "/mask"
        self.pubTopic      =  "/uncommon_points_topic"

        self.kinectPCData = None
        self.maskPCData   = None

        self.pub = rospy.Publisher(self.pubTopic, PointCloud2, queue_size=10)

    def run(self):
        
        print(" Listening @ /kinect/depth/points ")
        self.kinectPCData = rospy.wait_for_message(self.kinectPCTopic, PointCloud2) 
        print(" data rcvd ")
        
        print(" Listening @ /mask")
        self.maskPCData   = rospy.wait_for_message(self.maskPCtopic, PointCloud2)
        print(" data rcvd ")
                
        rospy.wait_for_service('add_two_ints')
        pointsCall = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        print(self.kinectPCData)
        print(self.maskPCData)
        pointsBool = pointsCall(self.kinectPCData, self.maskPCData)
        
        print(" Publishing @ /uncommon_points_topic")

        while not rospy.is_shutdown():
            self.pub.publish(pointsBool.outputPt)

        

if __name__ == "__main__":

    modifier = PointCloudModifier()
    modifier.run()