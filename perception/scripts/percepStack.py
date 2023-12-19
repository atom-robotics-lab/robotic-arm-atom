#!/usr/bin/env python3
import cv2
import rospy
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, PointField
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
        # Initialization of CV model
        self.bridge = CvBridge()

        # Subscribe to RGB and depth image topics using message_filters for time synchronization
        sub_rgb = message_filters.Subscriber("/kinect/color/image_raw", Image)
        sub_depth = message_filters.Subscriber("/kinect/depth/image_raw", Image)
        ts = message_filters.ApproximateTimeSynchronizer([sub_depth, sub_rgb], queue_size=1, slop=0.5)
        ts.registerCallback(self.callback)

        # ROS Publishers for publishing transforms and point cloud messages
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.mask_pub = rospy.Publisher("/mask", PointCloud2, queue_size=1)
        self.centroid_pub = rospy.Publisher("/centroid", PointCloud2, queue_size=1)

        # Paths and models
        self.model = YOLO('/home/aakshar/catkin_ws/src/flipkartGrid/perception/scripts/ml_models/best.pt')
        self.qrmodel = YOLO("/home/aakshar/catkin_ws/src/flipkartGrid/perception/scripts/ml_models/qr_detect.pt")
        self.confidence = 0.8

        # Image and shape variables
        self.rgb_image, self.depth_image = None, None
        self.rgb_shape, self.depth_shape = None, None

        # Flag indicating if the object is found
        self.found = False

    def rgb_callback(self, rgb_message):
        """
        Callback function for RGB image subscriber.

        Parameters:
        - rgb_message: ROS Image message containing RGB image data.
        """
        self.rgb_image = self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding="bgr8")
        self.rgb_shape = self.rgb_image.shape

    def depth_callback(self, depth_message):
        """
        Callback function for depth image subscriber.

        Parameters:
        - depth_message: ROS Image message containing depth image data.
        """
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding=depth_message.encoding)
        self.depth_shape = self.depth_image.shape

    def callback(self, depth_data, rgb_data):
        """
        Callback function for synchronized RGB and depth images.

        Parameters:
        - depth_data: ROS Image message containing depth image data.
        - rgb_data: ROS Image message containing RGB image data.
        """
        self.depth_callback(depth_data)
        self.rgb_callback(rgb_data)
        try:
            # Process RGB image to detect objects
            points, masks, boundingboxes = self.rgb_image_processing()

            # Process depth image to get depths of detected objects
            depths = self.depth_image_processing(points)

            # Find the index of the object with the minimum depth
            min_depth_index = depths.index(min(depths))

            # Extract RGB points within the bounding box of the object with minimum depth
            new_rgb_points = []
            for x in range(boundingboxes[min_depth_index][0], boundingboxes[min_depth_index][2]):
                for y in range(boundingboxes[min_depth_index][1], boundingboxes[min_depth_index][3]):
                    new_rgb_points.append((x, y))

            # Process the mask of the box to be picked
            self.process_box_mask(new_rgb_points)

            # Visualize the centroid of the detected object in the RGB image
            cv2.circle(self.rgb_image, points[min_depth_index], 8, (255, 0, 0), 3)
            cv2.imshow("point", self.rgb_image)
            cv2.waitKey(1)

            # Publish transforms of the box to be picked
            self.publish_transforms(self.find_XYZ(points[min_depth_index], depths[min_depth_index]))

            # Create PointCloud2 message for the centroid and publish
            self.publish_centroid_point_cloud(points[min_depth_index], depths[min_depth_index])

            rospy.sleep(7)
        except Exception as e:
            print("An error occurred", str(e))

    def rgb_image_processing(self):
        """
        Process RGB image using YOLO for object detection.

        Returns:
        - points: List of points (x, y) representing the detected objects in the RGB image.
        - masks: List of masks corresponding to the detected objects.
        - boundingboxes: List of bounding boxes (x1, y1, x2, y2) for the detected objects.
        """
        rgb_image = self.rgb_image
        points = []
        masks = []
        boundingboxes = []

        # Use YOLO model to predict objects in the RGB image
        results = self.model.predict(source=rgb_image, conf=self.confidence, show=True)

        # Process YOLO results
        for i in results[0].boxes.xywh:
            cv2.circle(rgb_image, (int(i[0]), int(i[1])), 5, (0, 0, 255), 2)
            points.append((int(i[0]), int(i[1])))

        for i in results[0].boxes.xyxy:
            boundingboxes.append((int(i[0]), int(i[1]), int(i[2]), int(i[3])))

        for mask in results[0].masks:
            masks.append(mask.xy[0])

        return points, masks, boundingboxes

    def depth_image_processing(self, points):
        """
        Process depth image to obtain depths of detected objects.

                # Process depth image to obtain depths of detected objects
        Parameters:
        - points: List of points (x, y) representing the detected objects in the RGB image.

        Returns:
        - depths: List of depths corresponding to the detected objects.
        """
        depth_array = np.array(self.depth_image, dtype=np.float32)
        depths = []

        for i in range(len(points)):
            x_center, y_center = int(points[i][1]), int(points[i][0])
            depths.append(depth_array[x_center, y_center])

        return depths

    def find_XYZ(self, point, depth):
        """
        Convert 2D point and depth to 3D coordinates (X, Y, Z).

        Parameters:
        - point: Tuple representing the 2D coordinates (x, y) of the point.
        - depth: Depth value at the given point.

        Returns:
        - xyz: Tuple representing the 3D coordinates (X, Y, Z) of the point.
        """
        fx, fy = [554.254691191187, 554.254691191187]
        cx, cy = [320.5, 240.5]

        X = depth * ((point[0] - cx) / fx)
        Y = depth * ((point[1] - cy) / fy)
        Z = depth

        return (X, Y, Z)

    def publish_centroid_point_cloud(self, point, depth):
        """
        Publish PointCloud2 message for the centroid of the detected object.

        Parameters:
        - point: Tuple representing the 2D coordinates (x, y) of the centroid.
        - depth: Depth value at the centroid.
        """
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
        point_cloud_msg = pc2.create_cloud(header, fields, [self.find_XYZ(point, depth)])
        self.centroid_pub.publish(point_cloud_msg)
        print("Published centroid")

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert Euler angles to quaternion.

        Parameters:
        - roll: Roll angle in radians.
        - pitch: Pitch angle in radians.
        - yaw: Yaw angle in radians.

        Returns:
        - quaternion: List representing the quaternion (qx, qy, qz, qw).
        """
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
            yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
            yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        return [qx, qy, qz, qw]

    def calculate_rotation_matrix(self, alpha, beta, gamma):
        """
        Calculate the rotation matrix from Euler angles.

        Parameters:
        - alpha: Roll angle in degrees.
        - beta: Pitch angle in degrees.
        - gamma: Yaw angle in degrees.

        Returns:
        - rotation_matrix: 3x3 rotation matrix.
        """
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
        """
        Convert 3D vector to Euler angles.

        Parameters:
        - x: x-component of the vector.
        - y: y-component of the vector.
        - z: z-component of the vector.

        Returns:
        - roll_deg, pitch_deg, yaw_deg: Euler angles in degrees.
        """
        # Ensure the vector is a NumPy array for convenient operations
        normal_vector = np.array([x, y, z])

        # Normalize the vector
        magnitude = np.linalg.norm(normal_vector)
        if magnitude != 0.0:
            normal_vector = normal_vector / magnitude

        # Calculate pitch (rotation around y-axis)
        yaw = np.arctan2(normal_vector[1], normal_vector[0])

        # Calculate roll (rotation around x-axis)
        pitch = np.arctan2(-normal_vector[2], ((((normal_vector[0]) ** 2) + ((normal_vector[1]) ** 2)) ** (0.5)))

        # Calculate yaw (rotation around z-axis)
        roll = np.arctan2(np.sin(yaw) * normal_vector[2], np.cos(yaw) * normal_vector[2])

        # Convert angles from radians to degrees
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)

        # Return the Euler angles
        return roll_deg, pitch_deg, yaw_deg

    def calculate_angles(self, angle_x, angle_y, angle_z):
        """
        Calculate angles from a unit vector.

        Parameters:
        - angle_x: Angle around x-axis in degrees.
        - angle_y: Angle around y-axis in degrees.
        - angle_z: Angle around z-axis in degrees.

        Returns:
        - angle_x_deg, angle_y_deg, angle_z_deg: Angles in degrees.
        """
        unit_vector = [angle_x, angle_y, angle_z]

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

        return angle_x_deg, angle_y_deg, angle_z_deg

    def alternative_rotation_matrix_to_quaternion(self, rotation_matrix):
        """
        Convert a rotation matrix to quaternion using an alternative method.

        Parameters:
        - rotation_matrix: 3x3 rotation matrix.

        Returns:
        - quaternion: List representing the quaternion (qx, qy, qz, qw).
        """
        # Ensure the rotation matrix is a valid rotation matrix
        assert np.allclose(np.dot(rotation_matrix, rotation_matrix.T), np.identity(3))

        # Calculate the quaternion directly
        qw = 0.5 * np.sqrt(1.0 + rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2])
        qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4.0 * qw)
        qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4.0 * qw)
        qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4.0 * qw)

        return np.array([qx, qy, qz, qw])

    def publish_transforms(self, xyz):
        """
        Publish transforms of the detected object.

        Parameters:
        - xyz: Tuple representing the 3D coordinates (X, Y, Z) of the detected object.
        """
        # Wait for the normal vector message
        vector = rospy.wait_for_message('/normals', Vector3, timeout=None)

        # Calculate Euler angles from the normal vector
        x, y, z = self.calculate_angles(vector.x, vector.y, vector.z)

        # Calculate rotation matrix from Euler angles
        rotation_matrix = self.calculate_rotation_matrix(x, y, z)

        # Convert rotation matrix to quaternion using an alternative method
        quaternion = self.alternative_rotation_matrix_to_quaternion(rotation_matrix)

        # Create a TransformStamped message
        t = TransformStamped()
        camera_trans = [-0.15, 0.612, 1.5]
        t.header.frame_id = "base_link"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "box"
        t.transform.translation.x = - (abs(xyz[1]) / xyz[1]) * (abs(xyz[1]) - (abs(xyz[1]) / xyz[1]) * abs(camera_trans[1]))
        t.transform.translation.y = - (abs(xyz[0]) / xyz[0]) * (abs(xyz[0]) + (abs(xyz[0]) / xyz[0]) * abs(camera_trans[0]))
        t.transform.translation.z = - abs(xyz[2]) + abs(camera_trans[2])
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        # Create TFMessage and publish
        tfm = tf2_msgs.msg.TFMessage([t])
        self.pub_tf.publish(tfm)

    def process_box_mask(self, mask):
        """
        Process the mask of the detected object and publish it as a point cloud.

        Parameters:
        - mask: List of points representing the mask of the detected object.
        """
        # Get depths of the points in the mask
        depths = self.depth_image_processing(mask)

        # Convert mask points to 3D coordinates
        mask_xyz = [self.find_XYZ(mask[i], depths[i]) for i in range(len(mask))]

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
    """
    Main function to initialize the ROS node and Perception class.
    """
    rospy.init_node("percepStack", anonymous=True)
    ps = Perception()
    rospy.sleep(1)


if __name__ == "__main__":
    main()
    rospy.spin()