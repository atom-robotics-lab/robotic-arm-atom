#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from ajgar_perception.srv import AddTwoInts

class PointCloudModifier:

    def __init__(self, topic):
        
        rospy.init_node('point_cloud_modifier')
        rospy.Subscriber(topic, PointCloud2, self.callback)

    
    def callback(self, points):
        rospy.wait_for_service('add_two_ints')
        pointsCall = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        pointsBool = pointsCall(points)
        print (pointsBool)


    def run(self):
        rospy.spin()

if __name__ == "__main__":
    modifier = PointCloudModifier("/kinect/depth/points")
    modifier.run()

