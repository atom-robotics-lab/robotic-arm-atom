#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time 

# Initialize ROS node
rospy.init_node('image_capture_node')

# Set the topic name to subscribe to (replace 'your_camera_topic' with the actual camera topic)
camera_topic = '/camera/rgb/image_color'

# Initialize a CvBridge object
bridge = CvBridge()

# Create a callback function to process the incoming image
def image_callback(msg):
    try:
        
        # Convert the ROS image message to a OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Save the image to a file
        image_filename = os.path.join('/home/aakshar/Pictures/final_pic', 'image_{}.jpg'.format(rospy.get_time()))
        cv2.imwrite(image_filename, cv_image)

        rospy.loginfo("Saved image to {}".format(image_filename))
        time.sleep(5)

         

    except Exception as e:
        rospy.logerr("Error processing the image: {}".format(str(e)))

# Subscribe to the camera topic and set the callback function
rospy.Subscriber(camera_topic, Image, image_callback)

# Specify the time interval (in seconds) between image captures
capture_interval = 0.002  # Set this to your desired interval

# Create a rate object to control the loop rate
time.sleep(5000)

# Run the ROS node
while not rospy.is_shutdown():
    rospy.spin()  # Continue running the node and callbacks
    rate.sleep(10)  # Control the loop rate
