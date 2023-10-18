#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray

class ArrayPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('array_publisher', anonymous=True)

        # Create a publisher for the /joint_angle_array topic
        self.array_publisher = rospy.Publisher('joint_angle_array', Int32MultiArray, queue_size=10)

        # Create an array (list)
        self.my_array = [1, 2, 3, 4, 5, 6]

    def publish_array(self):
        # Create an Int32MultiArray message to hold the array
        array_msg = Int32MultiArray(data=self.my_array)

        # Publish the array
        self.array_publisher.publish(array_msg)
        rospy.loginfo("Published array: %s" % array_msg.data)

if __name__ == '__main__':
    array_publisher = ArrayPublisher()
    
    # Add a brief delay to ensure the ROS node is fully initialized
    rospy.sleep(1)

    try:
        array_publisher.publish_array()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass