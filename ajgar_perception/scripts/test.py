#! /usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def main():
    rospy.init_node('test_node')

    # Wait for a message from the "/int_topic" topic
    int_msg = rospy.wait_for_message("/int_topic", Int32)

    # Extract the data from the message
    data = int_msg.data

    # Do something with the data...
    print(f"Data: {data}")

if __name__ == "__main__":
    main()
