#! /usr/bin/env python3

# ROS specific imports
import rospy
import rospkg
import tf2_msgs.msg
import geometry_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, PointField, Image
from ajgar_perception.srv import percepSrv, percepSrvResponse

# CV specific imports
import cv2
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge

from ajgar_sim_plugins.plugin_pneumatic_gripper.scripts import attach, detach

class Perception:
    def __init__(self) :

        # ROS node, service and YOLO model parameters 
        node    = "percepStack"
        srvnode = "percepStackSrv"
        tfPub   = "/tf"
        maskPub = "/mask"
        ImgPub  = "/processedImg"
        self.kinectColorSub = "/kinect/color/image_raw"
        self.kinectDepthSub = "/kinect/depth/image_raw"
        
        self.confidence = 0.4

        # global variables initialization 
        self.flag = 0  
        self.points = None
        self.depths = None
        self.tfValue = None
        self.rgbImage = None 
        self.maskValue = None
        self.min_depth_index = None
    
        # ROS node initialization
        rospy.init_node(node, anonymous=True)

        # ROS publishers and subscribers initialization
        self.rate = rospy.Rate(1)
        self.pub = rospy.Publisher(ImgPub, Image, queue_size=10)
        self.pub_tf = rospy.Publisher(tfPub, tf2_msgs.msg.TFMessage, queue_size=10)
        self.mask_pub = rospy.Publisher(maskPub, PointCloud2, queue_size=10)
  
        # YOLO model initialization
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("ajgar_perception")
        modelPath = package_path + "/scripts/ml_models/yolov8m-seg-custom.pt"
        self.model = YOLO(modelPath)

        # CvBridge initialization
        self.bridge = CvBridge()

        # ROS service initialization
        rospy.Service(srvnode, percepSrv, self.percepSrvCallback)
        print(" Ready with ROS service ")
    


    def percepSrvCallback(self, msg):
        ''' Callback function for ROS service '''

        self.flag = msg.flag        # set flag to 1 to publish tf and mask &  else set to 0 to process image
        frameSkipBuffer = 50        # number of frames to skip before processing the image
        frameSkipcnt = 0            # frame counter

        # wait for kinect depth data
        print(" Listening @ /kinect/depth/points ")
        while frameSkipcnt < frameSkipBuffer :                                  # skip frames to avoid processing the same image
            depthData = rospy.wait_for_message(self.kinectDepthSub, Image)
            frameSkipcnt = frameSkipcnt + 1
        print(" Data Rcvd ")
        
        frameSkipcnt = 0 

        # wait for kinect rgb data
        print(" Listening @ /kinect/colour/points")
        while frameSkipcnt < frameSkipBuffer :
            rgbData = rospy.wait_for_message(self.kinectColorSub, Image)
            frameSkipcnt = frameSkipcnt + 1 
        print(" Data Rcvd ")
        
        # process rgb and depth data
        depthImage = self.process_depth(depthData)
        self.rgbImage = self.process_rgb(rgbData)
        self.points, boundingboxes = self.rgb_image_processing(self.rgbImage)
        self.depths = self.depth_image_processing(self.points, depthImage)
        self.min_depth_index = self.depths.index(min(self.depths))
        new_rgb_points = []

        for x in range( boundingboxes[self.min_depth_index][0], boundingboxes[self.min_depth_index][2]):
            for y in range( boundingboxes[self.min_depth_index][1], boundingboxes[self.min_depth_index][3] ):
                new_rgb_points.append((x, y))
        
        #calculate mask of box
        self.maskValue = self.process_box_mask(new_rgb_points, depthImage)
        
        # draw centroid of box on rgb image
        cv2.circle(self.rgbImage, self.points[self.min_depth_index], 8, (255, 0, 0), 3)
        
        # calculate tf of box wrt to camera
        self.tfValue = self.find_XYZ( self.points[self.min_depth_index], self.depths[self.min_depth_index])
        print(" Img Processed  ")
        
        self.flag = 1 

        # return mask and tf values to service client
        tfValueBase = self.publish_transforms(self.tfValue, 0)

        return percepSrvResponse(self.maskValue, tfValueBase)
             

        
    def process_depth(self, depth_message):
        ''' Converts depth image to numpy array '''

        depth_image = self.bridge.imgmsg_to_cv2( depth_message, desired_encoding=depth_message.encoding )
        return depth_image
    
    
    def process_rgb(self, rgb_message):
        ''' Converts rgb image to numpy array '''

        rgb_image = self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding="bgr8")
        return rgb_image



    def rgb_image_processing(self, rgb_image):
        ''' Processes rgb image and returns bounding boxes and centroids of detected objects '''

        points = []
        boundingboxes = []

        results = self.model.predict(source=rgb_image, conf=self.confidence, show=False)

        for i in results[0].boxes.xywh:
            cv2.circle(rgb_image, (int(i[0]), int(i[1])), 5, (0, 0, 255), 2)
            points.append((int(i[0]), int(i[1])))

        for i in results[0].boxes.xyxy:
            boundingboxes.append((int(i[0]), int(i[1]), int(i[2]), int(i[3])))

        return points, boundingboxes



    def depth_image_processing(self, points, depth_image):
        ''' Processes depth image and returns depths of detected objects '''

        depth_array = np.array(depth_image, dtype=np.float32)
        depths = []

        for i in range(len(points)):
            x_center, y_center = int(points[i][1]), int(points[i][0])
            depths.append(depth_array[x_center, y_center])

        return depths



    def process_box_mask(self, mask, depth_image):
        ''' Processes depth image and returns mask of detected object '''
        
        depths = self.depth_image_processing(mask, depth_image)
        mask_xyz = []
        for i in range(len(mask)):
            mask_xyz.append(self.find_XYZ(mask[i], depths[i]))

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_depth_optical_frame"
        point_cloud_msg = pc2.create_cloud(header, fields, mask_xyz)
        
        print(" Mask Calculated ")
        return point_cloud_msg


    
    def find_XYZ(self, point, depth):
        ''' Calculates XYZ coordinates of a point in 3D space  '''

        fx, fy = [554.254691191187, 554.254691191187]
        cx, cy = [320.5, 240.5]
        X = depth * ((point[0] - cx) / fx)
        Y = depth * ((point[1] - cy) / fy)
        Z = depth
        return (X, Y, Z)



    def publish_transforms(self, xyz, mode):
        ''' Publishes tf of box wrt to camera '''

        camera_trans = [0.0, 0.40, 1.0]  # tf of base_link wrt to camera

        tf = geometry_msgs.msg.TransformStamped()
        tf.header.frame_id = "base_link"
        tf.header.stamp = rospy.Time.now()
        tf.child_frame_id = "box"

        tf.transform.translation.x = - (abs(xyz[1]) / xyz[1]) * (abs(xyz[1]) - (abs(xyz[1]) / xyz[1]) * abs(camera_trans[1]))
        tf.transform.translation.y = - (abs(xyz[0]) / xyz[0]) * (abs(xyz[0]) + (abs(xyz[0]) / xyz[0]) * abs(camera_trans[0]))
        tf.transform.translation.z = - abs(xyz[2]) + abs(camera_trans[2])

        tf.transform.rotation.x = 0
        tf.transform.rotation.y = 0
        tf.transform.rotation.z = 0
        tf.transform.rotation.w = 1

        if mode :
            tffm = tf2_msgs.msg.TFMessage([tf])
            self.pub_tf.publish(tffm)
        else :
            return [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]


    def main(self) :
        ''' Main function of the class '''

        while not rospy.is_shutdown():

            if ( self.flag ) :
                print("inside loop")
                img = self.bridge.cv2_to_imgmsg(self.rgbImage, encoding="passthrough")
                self.pub.publish(img)
                self.publish_transforms(self.tfValue, 1)
            self.rate.sleep()



if __name__ == "__main__":
    while not rospy.is_shutdown():
        perObject = Perception()
        perObject.main()
        rospy.spin()
