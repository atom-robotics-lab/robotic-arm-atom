#! /usr/bin/env python3

# import ROS libraries
import cv2 
import rospy
import tf2_msgs.msg
import message_filters
import geometry_msgs.msg
from   std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
from   sensor_msgs.msg import Image,PointCloud2, PointField
from   std_msgs.msg    import Int32 

# import Image Processing libraries
import numpy as np
from   ultralytics  import YOLO
from   cv_bridge    import CvBridge



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
        

        # OpenCV & YOLO Setup
        modelPath = '/home/arsenious/catkin_ws/src/flipkartGrid/ajgar_perception/scripts/ml_models/yolov8m-seg-custom.pt'
        self.model  = YOLO(modelPath)


        self.confidence = 0.4
        self.rgb_image, self.depth_image = None, None
        self.rgb_shape, self.depth_shape = None, None
        self.found = False
    
    def imageProcessBoolCallback(self, msg):
        if msg.data == 1:
            self.processBool = True
        else:
            self.processBool = False
        
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


    def callback(self,depth_data, rgb_data):

      if self.processBool == True:
        self.process_depth(depth_data)
        self.process_rgb(rgb_data)

        try:

            # points,masks,boundingboxes = self.rgb_image_processing()

            self.points, boundingboxes = self.rgb_image_processing()
            self.depths                = self.depth_image_processing(self.points)
            self.min_depth_index       = self.depths.index(min(self.depths))
            
            # self.extract_image(self.rgb_image,
            #                    boundingboxes[min_depth_index])

            # self.extract_image(np.array(self.depth_image, 
            #                             dtype=np.float32),
            #                             boundingboxes[min_depth_index])

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
            # self.publish_transforms(self.find_XYZ(self.points[self.min_depth_index],self.depths[self.min_depth_index]))
            print("image Processed")
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
	
	# tf of base_link wrt to camera 
        camera_trans = [0.0, 0.40, 1.0]
        
        
        tf = geometry_msgs.msg.TransformStamped()
        tf.header.frame_id = "base_link"
        tf.header.stamp = rospy.Time.now()
        tf.child_frame_id = "box"
        
        tf.transform.translation.x = - abs(xyz[1]) + abs(camera_trans[1]) 
        tf.transform.translation.y = - (abs(xyz[0])/xyz[0]) *(abs(xyz[0])  + (abs(xyz[0])/xyz[0]) * abs(camera_trans[0]))
        tf.transform.translation.z = - abs(xyz[2]) + abs(camera_trans[2])
        
        tf.transform.rotation.x = 0
        tf.transform.rotation.y = 0
        tf.transform.rotation.z = 0
        tf.transform.rotation.w = 1
        
        print("value : ", tf.transform.translation.x , tf.transform.translation.y , tf.transform.translation.z )
                
        tffm = tf2_msgs.msg.TFMessage([tf])
        
        self.pub_tf.publish(tffm)


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



if __name__=="__main__" :

    while not rospy.is_shutdown():
        perObject=Perception()
        rospy.spin()