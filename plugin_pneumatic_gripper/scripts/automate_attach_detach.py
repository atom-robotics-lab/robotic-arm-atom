#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ContactsState
import attach, detach
import time

count = 0
flag = False
obj2= None

def collision_callback(data):
    global count
    global flag
    global ob2

    for contact in data.states:
        collision_object_1, collision_object_2 = contact.collision1_name.split('-')

        if "ajgar::suction_1::suction_1_collision" == collision_object_1:
            count += 1
            obj2 = collision_object_2


def main():
    global flag
    while not flag:
        rospy.init_node('collision_subscriber', anonymous=True)
        rospy.Subscriber("/gazebo/collision/info", ContactsState, collision_callback)
        rospy.spin()
        if count > 100:
            flag = True
            count = 0

    if flag:
        attach.attach_links(obj2)  # Assuming you call attach_links without arguments
        time.sleep(10)  # Delay for 10 seconds
        detach.detach_links(obj2)  # Assuming you call detach_links without arguments

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass