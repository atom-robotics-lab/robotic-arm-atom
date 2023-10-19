#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from ajgar.srv import bool_service, bool_serviceRequest

class ArrayPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('array_publisher', anonymous=True)

        # Create a publisher for the /joint_angle_array topic
        self.array_publisher = rospy.Publisher('joint_angle_array', String, queue_size=10)

        self.list_of_arrays = "1,2,3,4,5,6"

    def publish_array(self):

        print(self.list_of_arrays)
        array_msg = String(data=self.list_of_arrays)

        # Publish the array
        self.array_publisher.publish(array_msg)
        rospy.loginfo("Published array: %s" % array_msg.data)

        self.call_service()

        rospy.sleep(1)

    def call_service(self):

        rospy.wait_for_service('bool_service')

        rospy.loginfo("service available")
        
        service_proxy = rospy.ServiceProxy('bool_service', bool_service)
        request = bool_serviceRequest()
        request.input = True
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