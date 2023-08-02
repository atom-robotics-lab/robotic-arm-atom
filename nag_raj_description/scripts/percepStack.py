#! /usr/bin/env python3
import cv2 
import rospy
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import tf2_ros
import tf2_msgs.msg
from ultralytics import YOLO
import numpy as np


class Perception:
    def __init__(self) -> None:
        # Initialisation of CV model

        self.bridge = CvBridge()

        sub_rgb = message_filters.Subscriber("/kinect/color/image_raw", Image)
        sub_depth = message_filters.Subscriber("/kinect/depth/image_raw", Image)
        ts = message_filters.ApproximateTimeSynchronizer([sub_depth, sub_rgb], queue_size=10, slop=0.5)
        ts.registerCallback(self.callback)

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        self.model=YOLO("/home/bhavay/catkin_ws/src/flipkartGrid/nag_raj_description/scripts/model.pt")
        self.confidence=0.45

        self.rgb_image, self.depth_image = None, None
        self.rgb_shape, self.depth_shape = None, None

        self.found=False
        
        
    def rgb_callback(self, rgb_message) :
        self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding = "bgr8")
        self.rgb_shape = self.rgb_image.shape

    
    def depth_callback(self, depth_message) :
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding=depth_message.encoding)
        self.depth_shape = self.depth_image.shape 


    def callback(self,depth_data, rgb_data):
        self.depth_callback(depth_data)
        self.rgb_callback(rgb_data)
        try:
            self.detect()
        except:
            pass

    def detect(self):
        points=self.rgb_image_processing()
        depth=self.depth_image_processing(points)
        print(points,depth)
    

    def rgb_image_processing(self):
        rgb_image = self.rgb_image 
        # cv2.imshow("rgb",rgb_image)
        # cv2.waitKey(1) 
        print(rgb_image.shape)
        points=[]
        results = self.model.predict(source=rgb_image,conf=self.confidence)
        for i in results[0].boxes.xywh:
            cv2.circle(rgb_image,(int(i[0]),int(i[1])),5,(0,0,255),2)
            points.append((int(i[0]),int(i[1])))
        cv2.imshow("points",rgb_image)
        cv2.waitKey(1) 
        return points
    

    def depth_image_processing(self,points):
        # print(self.depth_image.shape)
        depth_array = np.array(self.depth_image, dtype=np.float32)
        depth_image=self.depth_image
        depths=[]
        for i in range(len(points)):
            x_center, y_center = points[i][1], points[i][0]
            depths.append(depth_array[x_center, y_center])
            # cv2.circle(depth_image,(points[i][0], points[i][1]),5,(0,0,255),2)
        # cv2.imshow("points",depth_image)
        # cv2.waitKey(1) 
        return depths





def main():
    rospy.init_node("percepStack", anonymous=True)
    # try:
    ps = Perception()
    rospy.sleep(1)
    while True:
        pass
        # ps.detect()
        
    # except Exception as e:
        # ##print("Error:", str(e))    




if __name__=="__main__" :
    main()
    rospy.spin()