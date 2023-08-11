#!/usr/bin/env python3

import rospy
#from gazebo_msgs.msg import ContactsState
from std_msgs.msg import String
import attach, detach
import time

count = 0
flag = False
obj2= None

def collision_callback(data):
    #print("In func")
    global count
    global flag
    global ob2
    
    #print(data)
    objects = str(data).split(' ')[1].split('-')
    collision_object_1, collision_object_2 = objects[0][1:], objects[1][:-1]
    
    #print(f"{collision_object_1} and {collision_object_2} \n")
    if "ajgar::suction_1::suction_1_collision" == collision_object_1:
        count += 1
        obj2 = collision_object_2
        print(f"\nHit: {obj2} \nCount: {count}")
    
        if count > 100:
            flag = True
            count = 0
            automate(obj2)
            return

def automate(obj):
    print("Starting")
    attach.attach_links(obj)
    time.sleep(10)
    detach.detach_links(obj)

def main():
    print("Script started")
    rospy.init_node('collision_subscriber', anonymous=True)
    rospy.Subscriber("/gazebo/collision/info", String, collision_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass