#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg    import Int32
from std_srvs.srv    import Empty
from ajgar_perception.srv import AddTwoInts

class PointCloudModifier:

    def __init__(self):
       
        rospy.init_node('point_cloud_modifier')
        
        self.kinectPCTopic =  "/kinect/depth/points" 
        self.maskPCtopic   =  "/mask"
        self.pubTopic      =  "/uncommon_points_topic"
        self.imageProcessBoolTopic = "/imgProcessBool"

        self.kinectPCData = None
        self.maskPCData   = None
        self.pointBool    = None 
       
        self.cnt = 0 

        self.pub = rospy.Publisher(self.pubTopic, PointCloud2, queue_size=10)
        self.imageProcessBool = rospy.Publisher(self.imageProcessBoolTopic, Int32, queue_size=10)

    def run(self):
        
        print(" Listening @ /kinect/depth/points ")
        self.kinectPCData = rospy.wait_for_message(self.kinectPCTopic, PointCloud2) 
        print(" data rcvd ")
        
        print(" Listening @ /mask")
        self.maskPCData   = rospy.wait_for_message(self.maskPCtopic, PointCloud2)
        print(" data rcvd ")
                
        rospy.wait_for_service('add_two_ints')
        pointsCall = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        
        self.pointsBool = pointsCall(self.kinectPCData, self.maskPCData)
        
        
    def main(self):
        
        while not rospy.is_shutdown():
           
            self.run()

            print ("Clearing Octomap")
            rospy.wait_for_service('/clear_octomap')
            clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
            if (self.cnt == 60000) :
                    self.cnt = 0
                    break

            print(" Publishing @ /uncommon_points_topic")
            while not rospy.is_shutdown():
                self.pub.publish(self.pointsBool.outputPt)
                self.cnt += 1 
                if (self.cnt == 60000) :
                    self.cnt = 0
                    break 
            
            print(" Publishing @ /imgProcessBool : 1")
            while not rospy.is_shutdown():
                self.imageProcessBool.publish(1)
                self.cnt += 1 
                if (self.cnt == 60000) :
                        self.cnt = 0
                        break 
            

        

if __name__ == "__main__":

    modifier = PointCloudModifier()
    modifier.main()
