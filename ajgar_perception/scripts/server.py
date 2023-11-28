#!/usr/bin/env python3

from ajgar_perception.srv import percepSrv, octomapSrv, normalsSrv
from ajgar_core.srv import tfValueSrv
from sensor_msgs.msg import PointCloud2
from std_srvs.srv    import Empty

import rospy
import numpy as np


def server():
    
    pubTopic = "/uncommon_points_topic"
    cnt = 0

    rospy.init_node('masterNode')
    pub = rospy.Publisher(pubTopic, PointCloud2, queue_size=10)

    rospy.wait_for_service('percepStackSrv')
    print (" Request send to percepStackSrv, waiting for response ")
    
    percepSrvCall = rospy.ServiceProxy('percepStackSrv', percepSrv)
    percepSrvresponse = percepSrvCall(0)
    maskPCData = percepSrvresponse.maskValue
    tfArrayValue = percepSrvresponse.tfArray
    print(tfArrayValue)
    print(" Data Rcvd from percepStackSrv")

    kinectPCTopic =  "/kinect/depth/points"
    print(" Listening @ /kinect/depth/points ")
    kinectPCData = rospy.wait_for_message(kinectPCTopic, PointCloud2) 
    print(" data rcvd ")

    rospy.wait_for_service('octomapSrv')
    print (" Request send to octomapSrv, waiting for response")
    pointsCall = rospy.ServiceProxy('octomapSrv', octomapSrv)
    pointsBool = pointsCall(kinectPCData, maskPCData)
    print(" Data Rcvd from octomapSrv")

    rospy.wait_for_service('normals')
    print (" Request send to normalsSrv, waiting for response ")

    normalCall = rospy.ServiceProxy('normalsSrv',normalsSrv)
    points1 = np.array(maskPCData, dtype=np.float32).flatten()
    size1 = len(points1)
    points2 = np.array(tfArrayValue, dtype=np.float32).flatten()
    size2 = len(points2)
    normalSrvResponse = normalCall(points1,size1,points2,size2)
    orientation = normalSrvResponse.orientations
    #print(orientation)
    print("Data rcvd from normalsSrv")

    
    print ("Clearing Octomap")
    rospy.wait_for_service('/clear_octomap')

    clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
    clear_octomap()

 
    print(" Publishing @ /uncommon_points_topic")
    
    while not rospy.is_shutdown():
        pub.publish(pointsBool.outputPt)
        cnt += 1 
        if (cnt == 1) :
            cnt = 0
            break 

    print(" Publishing @ /uncommon_points_topic")


    # TODO : ingerate moveit with server.py

    rospy.wait_for_service('ikSolverSrv')
    print (" Request send to iksolverSrv, waiting for response")
    pointsCall = rospy.ServiceProxy('ikSolverSrv', tfValueSrv)
    pointsBool = pointsCall(tfArrayValue)
    print(" Data Rcvd from iksolverSrv")

if __name__ == "__main__":
    server()
