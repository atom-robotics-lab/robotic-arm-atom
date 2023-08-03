#! /usr/bin/env python3
import cv2 
import rospy
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import geometry_msgs.msg
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


    def find_centorid_of_largest_Surface(self,frame,xmin,xmax,ymin,ymax):
        
        # centroid_x = (xmin + xmax)/2
        # centroid_y = (ymin + ymax)/2
        pass

    def callback(self,depth_data, rgb_data):
        self.depth_callback(depth_data)
        self.rgb_callback(rgb_data)
        try:
            points,bounding_boxes = self.rgb_image_processing()
            depths = self.depth_image_processing(points)
            print(points,depths)
            # for i in range(len(points)):
            #     self.publish_transforms(self.find_transforms(points[i],depths[i]))
            
            min_depth_index = depths.index(min(depths))
            print(points[min_depth_index])
            self.publish_transforms(self.find_transforms(points[min_depth_index],depths[min_depth_index]))
        except:
            print("An error occoured")
        
        # coords = bounding_boxes[min_depth_index]
        # x,y,radius = self.find_centorid_of_largest_Surface(self.rgb_image,coords[0],coords[1],coords[2],coords[3])
        
        # x1, y1 = x - radius, y #Left
        # x2, y2 = x, y - radius #Top
        # x3, y3 = x + radius, y #Right
        # x4, y4 = x, y + radius #Bottom
        
        # bounding_points = [(x,y),(x1,y1),(x2,y2),(x3,y3),(x4,y4)]
        # new_depths = self.depth_image_processing(bounding_points)
        
        # new_points_xyz = []
        # for i in range(5):
        #     point = self.find_transforms(bounding_points[i],new_depths[i])
        #     new_points_xyz.append(point)
            
        # print(new_points_xyz)
    

    def rgb_image_processing(self):
        rgb_image = self.rgb_image 
        # cv2.imshow("rgb",rgb_image)
        # cv2.waitKey(1) 
        print(rgb_image.shape)
        points=[]
        results = self.model.predict(source=rgb_image,conf=self.confidence)
        bounding_boxes=[]
        for i in results[0].boxes.xywh:
            cv2.circle(rgb_image,(int(i[0]),int(i[1])),5,(0,0,255),2)
            points.append((int(i[0]),int(i[1])))
        for i in results[0].boxes.xyxy:
            bounding_boxes.append((int(i[0]),int(i[1]),int(i[2]),int(i[3])))
        cv2.imshow("points",rgb_image)
        cv2.waitKey(1) 
        return points,bounding_boxes
    

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

    def find_transforms(self,point,depth):
        fx, fy = [554.254691191187, 554.254691191187]
        cx, cy = [320.5, 240.5]

        #tf = TransformFrames()
        tf_buffer = tf2_ros.Buffer()
        tf_listener=tf2_ros.TransformListener(tf_buffer)

        #pose_array = PoseArray(header=Header(frame_id = "camera_depth_frame2", stamp = rospy.Time(0)))

        X = depth * ((point[0]-cx)/fx)
        Y = depth * ((point[1]-cy)/fy)
        Z = depth
        print(X , Y , Z )
        return [X,Y,Z]
    
    def publish_transforms(self,xyz):
        t = geometry_msgs.msg.TransformStamped()
        t.header.frame_id = "base_link"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "Box"
        t.transform.translation.x = xyz[0]
        t.transform.translation.y = xyz[1]
        t.transform.translation.z = xyz[2]
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1            
        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)



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