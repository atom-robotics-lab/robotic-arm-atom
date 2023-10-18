#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray
from ajgar.srv import bool_service

int_array1 = [1, 2, 3, 4, 5, 6]
int_array2 = [7, 8, 9, 10, 11, 12]
int_array3 = [13, 14, 15, 16, 17, 18]
int_array4 = [19, 20, 21, 22, 23, 24]
int_array5 = [25, 26, 27, 28, 29, 30]
int_array6 = [31, 32, 33, 34, 35, 36]

class ArrayPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('array_publisher', anonymous=True)

        # Create a publisher for the /joint_angle_array topic
        self.array_publisher = rospy.Publisher('joint_angle_array', Int32MultiArray, queue_size=10)

        self.list_of_arrays = [int_array1, int_array2, int_array3, int_array4, int_array5, int_array6]

    def publish_array(self):
        i = 0
        for arr in self.list_of_arrays:
            print(arr)
            print(self.list_of_arrays[i])
            print(i)
            # Create an Int32MultiArray message to hold the array
            array_msg = Int32MultiArray(data=self.list_of_arrays[i])

            # Publish the array
            self.array_publisher.publish(array_msg)
            rospy.loginfo("Published array: %s" % array_msg.data)
            self.call_service()
            i+=1
            rospy.sleep(1)

    def call_service(self):

        rospy.wait_for_service('bool_service')
        rospy.loginfo("service available")
        service_proxy = rospy.ServiceProxy('bool_service', bool_service)
        request = {'field_name': True}
        response = service_proxy(request)
        rospy.loginfo("Service response: %s", response)
        rospy.loginfo("service has been completed")
        return

if __name__ == '__main__':
    array_publisher = ArrayPublisher()
    
    # Add a brief delay to ensure the ROS node is fully initialized
    rospy.sleep(1)

    try:
        array_publisher.publish_array()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass