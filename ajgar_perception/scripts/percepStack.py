#! /usr/bin/env python3

# import ROS libraries
import cv2 
import rospy
import tf2_msgs.msg
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header, String
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_geometry_msgs import do_transform_pose
import tf2_ros
import tf2_msgs.msg
from ultralytics import YOLO
import numpy as np
from pathlib import Path


class Perception:

    def __init__(self) -> None:

        # ROS Setup
        node    = "percepStack"
        tfPub   = "/tf"
        maskPub = "/mask"
        kinectColorSub   = "/kinect/color/image_raw"
        kinectDepthSub   = "/kinect/depth/image_raw"
        self.pub = rospy.Publisher('processedImg', Image, queue_size=10)

        self.processBool = True

        rospy.init_node(node, anonymous=True)
        
        self.pub_tf   = rospy.Publisher(tfPub   , tf2_msgs.msg.TFMessage, queue_size=1)
        self.mask_pub = rospy.Publisher(maskPub , PointCloud2,queue_size=1)
        rospy.Subscriber("imgProcessBool", Int32, self.imageProcessBoolCallback)
        sub_rgb       = message_filters.Subscriber(kinectColorSub, Image)
        sub_depth     = message_filters.Subscriber(kinectDepthSub, Image)

        self.bridge = CvBridge()
        ts = message_filters.ApproximateTimeSynchronizer([sub_depth, sub_rgb], 
                                                          queue_size=1, 
                                                          slop=0.5 )
        ts.registerCallback(self.callback)

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.mask_pub=rospy.Publisher("/mask",PointCloud2,queue_size=1)
        self.centroid_pub=rospy.Publisher("/centroid",PointCloud2, queue_size=1)
        self.pose_pub=rospy.Publisher("/pose",PoseStamped,queue_size=1)
        sub_detect=rospy.Subscriber("/detect",String,self.detect)

        self.full_path = f'{Path.cwd()}' 

        self.model=YOLO('/home/bhavay/catkin_ws/src/flipkartGrid/ajgar_perception/scripts/ml_models/final_seg_model.pt')
        self.confidence=0.4
        self.rgb_image, self.depth_image = None, None
        self.rgb_shape, self.depth_shape = None, None
        
        
    def process_rgb(self, rgb_message) :
        self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_message, 
                                                   desired_encoding = "bgr8")
        self.rgb_shape = self.rgb_image.shape

    
    def process_depth(self, depth_message) :
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding = depth_message.encoding)
        self.depth_shape = self.depth_image.shape 

    
    def extract_image(self,image,boundingbox):
        region=image[boundingbox[1]:boundingbox[3],
                     boundingbox[0]:boundingbox[2]]
        return region


    def callback(self, depth_data, rgb_data):
        self.depth_callback(depth_data)
        self.rgb_callback(rgb_data)

    
    def detect(self, message):
        try:

            # points,masks,boundingboxes = self.rgb_image_processing()

            self.points, boundingboxes = self.rgb_image_processing()
            self.depths                = self.depth_image_processing(self.points)
            self.min_depth_index       = self.depths.index(min(self.depths))
            
            # self.extract_image(self.rgb_image,
            #                    boundingboxes[min_depth_index])

            # Extract the bounding box images of the box to be picked
            # new_rgb=self.extract_image(self.rgb_image,boundingboxes[min_depth_index])
            # new_depth=self.extract_image(np.array(self.depth_image, dtype=np.float32),boundingboxes[min_depth_index])

            new_rgb_points=[]

            for x in range(boundingboxes[self.min_depth_index][0],boundingboxes[self.min_depth_index][2]):
                for y in range(boundingboxes[self.min_depth_index][1],boundingboxes[self.min_depth_index][3]):
                    new_rgb_points.append((x,y))
            
            # Process the mask of the box to be picked
            self.process_box_mask(new_rgb_points)


            cv2.circle(self.rgb_image, self.points[self.min_depth_index],8,(255,0,0),3)
            
            #cv2.imshow("point",self.rgb_image)
            #cv2.waitKey(1)

            # Publish transforms of box to be picked  
            self.publish_transforms(self.find_XYZ(points[min_depth_index],depths[min_depth_index]))
            
            # Publish pose of box to be picked  
            self.publish_pose(self.find_XYZ(points[min_depth_index],depths[min_depth_index]))


            # Publish centroid of the box to picked

            # Define point fields
            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            # Create PointCloud2 message
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "camera_depth_optical_frame"
            point_cloud_msg = pc2.create_cloud(header, fields, [self.find_XYZ(points[min_depth_index],depths[min_depth_index]),])
            self.centroid_pub.publish(point_cloud_msg)
            print("Published centroid")

        except Exception as e:
            print("An error occoured",str(e))

      else:
            img = self.bridge.cv2_to_imgmsg(self.rgb_image, encoding="passthrough")
            self.pub.publish(img)
            self.publish_transforms(self.find_XYZ(self.points[self.min_depth_index], self.depths[self.min_depth_index]))

          
    

    def rgb_image_processing(self):
        rgb_image     = self.rgb_image 
        points        = []
        # masks         = []
        boundingboxes = []

        results = self.model.predict(source = rgb_image,
                                     conf=self.confidence,
                                     show=False)

        for i in results[0].boxes.xywh:
            cv2.circle(rgb_image,(int(i[0]),int(i[1])),5,(0,0,255),2)
            points.append((int(i[0]),int(i[1])))

        for i in results[0].boxes.xyxy:
            boundingboxes.append((int(i[0]),int(i[1]),int(i[2]),int(i[3])))
        
        # for mask in results[0].masks:
        #     masks.append(mask.xy[0])

        # return points,masks,boundingboxes
        return points,boundingboxes
    

    def depth_image_processing(self, points):
        depth_array = np.array(self.depth_image, dtype=np.float32)
        depths = []

        for i in range(len(points)):
            x_center, y_center = int(points[i][1]), int(points[i][0])
            depths.append(depth_array[x_center, y_center])

        return depths


    def find_XYZ(self,point,depth):
        fx, fy = [554.254691191187, 554.254691191187]
        cx, cy = [320.5, 240.5]

        X = depth * ((point[0]-cx)/fx)
        Y = depth * ((point[1]-cy)/fy)
        Z = depth
        return (X,Y,Z)
    

    def publish_transforms(self,xyz):
        t = TransformStamped()
        t.header.frame_id = "camera_depth_optical_frame"
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


    def publish_pose(self,xyz):

        # Create a tf2 buffer
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        pose = PoseStamped()
        pose.header.frame_id = "camera_depth_optical_frame"
        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0  # Default orientation

        try:
            # Lookup the transform from Frame A to Frame B
            transform = tf_buffer.lookup_transform("world", "camera_depth_optical_frame", rospy.Time(0), rospy.Duration(1.0))

            # Transform the position to Frame B
            new_pose = do_transform_pose(pose, transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Error during transformation: %s", e)

        self.pose_pub.publish(new_pose)


    def process_box_mask(self,mask):
        depths=self.depth_image_processing(mask)
        mask_xyz=[]
        for i in range(len(mask)):
            mask_xyz.append(self.find_XYZ(mask[i],depths[i]))
        
        # Define point fields
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Create PointCloud2 message
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_depth_optical_frame"
        point_cloud_msg = pc2.create_cloud(header, fields, mask_xyz)
        self.mask_pub.publish(point_cloud_msg)
        print("Published mask")



def main():
    rospy.init_node("percepStack", anonymous=True)
    ps = Perception()
    rospy.sleep(1)
    while not rospy.is_shutdown():
        rospy.spin() 
        # ps.detect()
        
    # except Exception as e:
        # print("Error:", str(e))    






if __name__=="__main__" :

    while not rospy.is_shutdown():
        perObject=Perception()
        rospy.spin()
