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
from scipy.spatial.transform import Rotation
from numpy.linalg import norm





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

            new_rgb_points=[]
            for x in range(boundingboxes[min_depth_index][0],boundingboxes[min_depth_index][2]):
                for y in range(boundingboxes[min_depth_index][1],boundingboxes[min_depth_index][3]):
                    new_rgb_points.append((x,y))
            
            # Process the mask of the box to be picked
            self.process_box_mask(new_rgb_points)

            cv2.circle(self.rgb_image,points[min_depth_index],8,(255,0,0),3)
            cv2.imshow("point",self.rgb_image)
            cv2.waitKey(1)
            
            # Publish transforms of box to be picked  
            self.publish_transforms(self.find_XYZ(points[min_depth_index],depths[min_depth_index]))
            
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
        depth_array = np.array(self.depth_image, dtype=np.float32)
        depths=[]
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
        # print("XYZ:",X , Y , Z )
        return (X,Y,Z)



    def get_quaternion_from_euler(self,roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, (qw)]


    
    def calculate_rotation_matrix(self, alpha, beta, gamma):
        # Convert angles to radians
        alpha = np.radians(alpha)
        beta = np.radians(beta)
        gamma = np.radians(gamma)

        # Create rotation object
        r = Rotation.from_euler('zyx', [gamma, beta, alpha], degrees=True)

        # Get the rotation matrix
        rotation_matrix = r.as_matrix()

        return rotation_matrix
        


    def Vector_to_Euler(self, x, y, z):
            # Ensure the vector is a NumPy array for convenient operations
            normal_vector = np.array([x, y, z])

            # Normalize the vector
            magnitude = np.linalg.norm(normal_vector)
            if magnitude != 0.0:
                normal_vector = normal_vector / magnitude

            # Calculate pitch (rotation around y-axis)
            yaw = np.arctan2(normal_vector[1],normal_vector[0])

            # Calculate roll (rotation around x-axis)
            pitch = np.arctan2(-normal_vector[2], ((((normal_vector[0])**2)+((normal_vector[1])**2))**(0.5)))

            # Calculate yaw (rotation around z-axis)
            roll = np.arctan2( np.sin(yaw)*normal_vector[2], np.cos(yaw)*normal_vector[2])

            # Convert angles from radians to degrees
            roll_deg = np.degrees(roll)
            pitch_deg = np.degrees(pitch)
            yaw_deg = np.degrees(yaw)

            # Return the Euler angles
            return roll_deg, pitch_deg, yaw_deg



    def calculate_angles(self, angle_x, angle_y, angle_z):
        unit_vector=[angle_x, angle_y, angle_z]
        # Ensure the input vector is a unit vector
        assert np.isclose(norm(unit_vector), 1.0)

        # Calculate angles in radians
        angle_x = np.arccos(np.dot(unit_vector, [1, 0, 0]))
        angle_y = np.arccos(np.dot(unit_vector, [0, 1, 0]))
        angle_z = np.arccos(np.dot(unit_vector, [0, 0, 1]))

        # Convert angles to degrees if needed
        angle_x_deg = np.degrees(angle_x)
        angle_y_deg = np.degrees(angle_y)
        angle_z_deg = np.degrees(angle_z)

        return (angle_x_deg, angle_y_deg, angle_z_deg)
        


    def alternative_rotation_matrix_to_quaternion(self, rotation_matrix):
        # Ensure the rotation matrix is a valid rotation matrix
        assert np.allclose(np.dot(rotation_matrix, rotation_matrix.T), np.identity(3))

        # Calculate the quaternion directly
        qw = 0.5 * np.sqrt(1.0 + rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2])
        qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4.0 * qw)
        qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4.0 * qw)
        qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4.0 * qw)

        return np.array([qx, qy, qz, qw])



    def publish_transforms(self,xyz):
        vector=rospy.wait_for_message('/normals', Vector3, timeout=None)
        x, y, z = self.calculate_angles((vector.x), (vector.y), (vector.z))
        Rotation_matrix=self.calculate_rotation_matrix(x, y, z)
        Quaternion=self.alternative_rotation_matrix_to_quaternion(Rotation_matrix)
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
