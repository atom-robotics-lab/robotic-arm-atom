#! /usr/bin/env python3
import cv2 
import rospy
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, PoseStamped
from geometry_msgs.msg import Vector3 
from tf2_geometry_msgs import do_transform_pose
import transformations
import tf2_ros
import transforms3d as tf3d
import tf2_msgs.msg
from ultralytics import YOLO
import numpy as np
from pathlib import Path
import math






class Perception:
    def __init__(self) -> None:
        # Initialisation of CV model

        self.bridge = CvBridge()

        sub_rgb = message_filters.Subscriber("/kinect/color/image_raw", Image)
        sub_depth = message_filters.Subscriber("/kinect/depth/image_raw", Image)
        ts = message_filters.ApproximateTimeSynchronizer([sub_depth, sub_rgb], queue_size=1, slop=0.5)
        ts.registerCallback(self.callback)

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.mask_pub=rospy.Publisher("/mask",PointCloud2,queue_size=1)
        self.centroid_pub=rospy.Publisher("/centroid",PointCloud2, queue_size=1)
        self.pose_pub=rospy.Publisher("/pose",PoseStamped,queue_size=1)

        self.full_path = f'{Path.cwd()}' 

        self.model=YOLO('/home/aakshar/Downloads/best.pt')
        self.qrmodel = YOLO("/home/aakshar/catkin_ws/src/flipkartGrid/perception/scripts/ml_models/qr_detect.pt")
        self.confidence=0.8

        self.rgb_image, self.depth_image = None, None
        self.rgb_shape, self.depth_shape = None, None

        self.found=False
        
        
    def rgb_callback(self, rgb_message) :
        self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding = "bgr8")
        self.rgb_shape = self.rgb_image.shape

    
    def depth_callback(self, depth_message) :
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding=depth_message.encoding)
        self.depth_shape = self.depth_image.shape 

    
    def extract_image(self,image,boundingbox):
        region=image[boundingbox[1]:boundingbox[3],boundingbox[0]:boundingbox[2]]
        # cv2.imshow("reigon",region)
        # cv2.waitKey(0)
        return region


    def callback(self,depth_data, rgb_data):
        self.depth_callback(depth_data)
        self.rgb_callback(rgb_data)
        try:

            points,masks,boundingboxes = self.rgb_image_processing()
            # print(masks)
            depths = self.depth_image_processing(points)
            # print(points,depths)
            
            min_depth_index = depths.index(min(depths))
            # print(points[min_depth_index])

            # Process the mask of the box to be picked
            # self.process_box_mask(masks[min_depth_index])

            # Extract the bounding box images of the box to be picked
            # new_rgb=self.extract_image(self.rgb_image,boundingboxes[min_depth_index])
            # new_depth=self.extract_image(np.array(self.depth_image, dtype=np.float32),boundingboxes[min_depth_index])

            new_rgb_points=[]
            for x in range(boundingboxes[min_depth_index][0],boundingboxes[min_depth_index][2]):
                for y in range(boundingboxes[min_depth_index][1],boundingboxes[min_depth_index][3]):
                    new_rgb_points.append((x,y))
            
            # Process the mask of the box to be picked
            self.process_box_mask(new_rgb_points)


            cv2.circle(self.rgb_image,points[min_depth_index],8,(255,0,0),3)
            cv2.imshow("point",self.rgb_image)
            cv2.waitKey(1)

            #wait_for_msg function laga for quaternion
            
            
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
            rospy.sleep(7)
        except Exception as e:
            print("An error occoured",str(e))
    

    def rgb_image_processing(self):
        rgb_image = self.rgb_image 
        # cv2.imshow("RGB Image",rgb_image)
        # cv2.waitKey(1) 
        # print("RGB Image shape:",rgb_image.shape)
        points=[]
        masks=[]
        boundingboxes=[]

        results = self.model.predict(source=rgb_image,conf=self.confidence,show=True)


        for i in results[0].boxes.xywh:
            cv2.circle(rgb_image,(int(i[0]),int(i[1])),5,(0,0,255),2)
            points.append((int(i[0]),int(i[1])))

        for i in results[0].boxes.xyxy:
            boundingboxes.append((int(i[0]),int(i[1]),int(i[2]),int(i[3])))
        
        for mask in results[0].masks:
            masks.append(mask.xy[0])
        
        # cv2.imshow("points",rgb_image)
        # cv2.waitKey(1) 
        return points,masks,boundingboxes
    

    def depth_image_processing(self,points):
        # print("Depth Image Shape:",self.depth_image.shape)
        depth_array = np.array(self.depth_image, dtype=np.float32)
        depths=[]
        for i in range(len(points)):
            x_center, y_center = int(points[i][1]), int(points[i][0])
            depths.append(depth_array[x_center, y_center])
            # cv2.circle(depth_image,(points[i][0], points[i][1]),5,(0,0,255),2)

        # Show the depth image
        # cv2.imshow("Depth Image",depth_image)
        # cv2.waitKey(1) 
        return depths


    def find_XYZ(self,point,depth):
        fx, fy = [554.254691191187, 554.254691191187]
        cx, cy = [320.5, 240.5]

        # tf = TransformFrames()
        # tf_buffer = tf2_ros.Buffer()
        # tf_listener=tf2_ros.TransformListener(tf_buffer)
        # pose_array = PoseArray(header=Header(frame_id = "camera_depth_frame2", stamp = rospy.Time(0)))

        X = depth * ((point[0]-cx)/fx)
        Y = depth * ((point[1]-cy)/fy)
        Z = depth
        # print("XYZ:",X , Y , Z )
        return (X,Y,Z)
    
    def get_quaternion_from_euler(self,roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, (qw)]
    
    

    # def rotate_quaternion(self, original_quaternion,):
    #     # Create a TransformStamped message with the original quaternion
    #     axis_index=2
    #     angle_degrees=180
    #     transform_msg = TransformStamped()
    #     transform_msg.transform.rotation = original_quaternion

    #     # Create a tf2_ros buffer and transform listener
    #     buffer = tf2_ros.Buffer()
    #     listener = tf2_ros.TransformListener(buffer)

    #     # Wait for the transformation from the source frame to the target frame
    #     source_frame = "camera_depth_optical_frame"
    #     target_frame = "base_link"
    #     rospy.sleep(1.0)  # Add a delay to make sure the transform is available

    #     try:
    #         # Lookup the transform
    #         transform = buffer.lookup_transform(target_frame, source_frame, rospy.Time())

    #         # Rotate the quaternion by the specified angle around the specified axis
    #         rotation = transformations.quaternion_about_axis(angle_degrees, [1 if i == axis_index else 0 for i in range(3)])

    #         # Apply the rotation to the quaternion
    #         rotated_quaternion = transformations.quaternion_multiply(transform_msg.transform.rotation, rotation)

    #         # Update the transform with the rotated quaternion
    #         transform_msg.transform.rotation = rotated_quaternion

    #         return transform_msg.transform.rotation
    #     except tf2_ros.LookupException as e:
    #         rospy.logwarn("Transform lookup failed: %s", e)
    #         return None
        

    def rotate_z_axis(self, x, y, z):
        vector=[x, y, z]
        # Convert angle to radians
        angle_radians = np.radians(180)

        # Create the rotation matrix for a rotation about the Z-axis
        rotation_matrix = np.array([
            [np.cos(angle_radians), -np.sin(angle_radians), 0],
            [np.sin(angle_radians), np.cos(angle_radians), 0],
            [0, 0, 1]
        ])

        # Apply the rotation to the vector
        rotated_vector = np.dot(rotation_matrix, vector)

        return rotated_vector
    def vector_to_euler(self, x, y, z):
            # Ensure the vector is a NumPy array for convenient operations
            normal_vector = np.array([x, y, z])

            # Normalize the vector
            magnitude = np.linalg.norm(normal_vector)
            if magnitude != 0.0:
                normal_vector = normal_vector / magnitude

            # Calculate pitch (rotation around y-axis)
            pitch = np.arcsin(-normal_vector[1])

            # Calculate roll (rotation around x-axis)
            roll = np.arctan2(normal_vector[0], normal_vector[2])

            # Calculate yaw (rotation around z-axis)
            yaw = np.arctan2(normal_vector[1], normal_vector[0])

            # Convert angles from radians to degrees
            roll_deg = np.degrees(roll)
            pitch_deg = np.degrees(pitch)
            yaw_deg = np.degrees(yaw)

            # Return the Euler angles
            return roll_deg, pitch_deg, yaw_deg






    def publish_transforms(self,xyz):
        vector=rospy.wait_for_message('/normals', Vector3, timeout=None)
        x, y, z = self.Vector_to_Euler((vector.x), (vector.y), (vector.z))

        #x, y, z = self.rotate_z_axis(x, y, z)

        Quaternion = self.get_quaternion_from_euler(np.radians(x), np.radians(y), np.radians(z))
        #New_Quaternion = self.rotate_quaternion(self.get_quaternion_from_euler(Euler.x, Euler.z, -Euler.y))
        t = TransformStamped()
        camera_trans = [-0.15, 0.612, 1.5]
        t.header.frame_id = "base_link"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "box"
        t.transform.translation.x = - (abs(xyz[1]) / xyz[1]) * (abs(xyz[1]) - (abs(xyz[1]) / xyz[1]) * abs(camera_trans[1]))
        t.transform.translation.y = - (abs(xyz[0]) / xyz[0]) * (abs(xyz[0]) + (abs(xyz[0]) / xyz[0]) * abs(camera_trans[0]))
        t.transform.translation.z = - abs(xyz[2]) + abs(camera_trans[2])
        t.transform.rotation.x = Quaternion[0]
        t.transform.rotation.y = Quaternion[1]
        t.transform.rotation.z = Quaternion[2]
        t.transform.rotation.w = Quaternion[3]
        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)
    
    # def publish_transforms(self,xyz):

    #     t = TransformStamped()
    #     camera_trans = [-0.15, 0.612, 1.5]
    #     t.header.frame_id = "base_link"
    #     t.header.stamp = rospy.Time.now()
    #     t.child_frame_id = "Box"
    #     t.transform.translation.x = xyz[0]
    #     t.transform.translation.y = xyz[1]
    #     t.transform.translation.z = xyz[2]

    #     t.transform.translation.x = - (abs(xyz[1]) / xyz[1]) * (abs(xyz[1]) - (abs(xyz[1]) / xyz[1]) * abs(camera_trans[1]))
    #     t.transform.translation.y = - (abs(xyz[0]) / xyz[0]) * (abs(xyz[0]) + (abs(xyz[0]) / xyz[0]) * abs(camera_trans[0]))
    #     t.transform.translation.z = - abs(xyz[2]) + abs(camera_trans[2])

    #     t.transform.rotation.x =  0.001
    #     t.transform.rotation.y =  0.001
    #     t.transform.rotation.z = -0.479
    #     t.transform.rotation.w =  0.878        
    #     tfm = tf2_msgs.msg.TFMessage([t])
    #     self.pub_tf.publish(tfm)

    def publish_pose(self,xyz):

        # Create a tf2 buffer
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        pose = PoseStamped()
        pose.header.frame_id = "camera_depth_optical_frame"
        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        pose.pose.orientation.x = 0.001
        pose.pose.orientation.y = 0.001
        pose.pose.orientation.z = -0.479
        pose.pose.orientation.w = 0.878  # Default orientation

        try:
            # Lookup the transform from Frame A to Frame B
            transform = tf_buffer.lookup_transform("base_link", "camera_depth_optical_frame", rospy.Time(0), rospy.Duration(1.0))

            # Transform the position to Frame B
            new_pose = do_transform_pose(pose, transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Error during transformation: %s", e)

        self.pose_pub.publish(new_pose)


    # def cost_analysis(self, depths, points ,masks):
    #     """
    #     Mark the center of the bounding box with the lowest depth in blue.

    #     Parameters:
    #     - depths: List of depths corresponding to the detected objects.
    #     - points: List of (x, y) coordinates representing the centers of detected bounding boxes.

    #     Returns:
    #     - index_of_min_depth: Index of the center with the lowest depth.
    #     """
    #     cost=[]
    #     QR=[1]
    #     MF_depth=1
    #     for i in range(len(masks)):
    #         mask = masks[i]

    #         # Create a masked image using the bounding box
    #         masked_image = cv2.fillPoly(np.zeros_like(self.rgb_image), [np.array(mask)], (255, 255, 255))
    #         gray_masked = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)

    #         # Perform QR code detection (replace with your actual QR code scanning logic)
    #         qr_code_found = detect_qr_code(gray_masked)

    #         if qr_code_found:
    #             result = 2  # QR code found, update result and break out of the loop
    #             break
            
    #         cost.append(len(masks[i])*(QR))/(depths[i]*MF_depth)




        # # Find the index of the center with the max cost
        # index_of_max_cost = cost.index(max(cost))

        # # Mark the center with the lowest depth in blue
        # cv2.circle(self.rgb_image, points[index_of_max_cost], 8, (0, 0, 255), 3)

        # # Display the annotated image (optional)
        # cv2.imshow("Annotated Image", self.rgb_image)
        # cv2.waitKey(1)

        # return index_of_max_cost

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
    # try:
    ps = Perception()
    rospy.sleep(1)
        # ps.detect()
        
    # except Exception as e:
        # print("Error:", str(e))    




if __name__=="__main__" :
    main()
    rospy.spin()
