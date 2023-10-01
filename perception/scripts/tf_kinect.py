#!/usr/bin/env python

import rospy
import tf2_ros
from tf2_geometry_msgs import PointStamped, QuaternionStamped
from geometry_msgs.msg import TransformStamped

def tf_callback(msg):
    try:
        # Transform the camera's orientation to the desired frame (e.g., "world")
        trans = tf_buffer.lookup_transform("world", "camera_link", rospy.Time(0), rospy.Duration(1.0))
        # Extract the orientation quaternion from the transform
        orientation = trans.transform.rotation
        # Print the orientation as a quaternion
        rospy.loginfo(f"Orientation (Quaternion): {orientation}")
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr("TF lookup failed: %s", str(e))

if __name__ == '__main__':
    rospy.init_node('kinect_orientation_listener')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(10)  # Update rate (Hz)

    while not rospy.is_shutdown():
        try:
            # Listen to the TF data and get the orientation of the Kinect camera
            tf_callback(None)
        except rospy.ROSInterruptException:
            pass
        rate.sleep()
