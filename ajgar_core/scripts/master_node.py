#!/usr/bin/env python3

import rospy
import time

from ajgar_perception.srv import Data, Target 




class masterNode(object) :
      
      def __init__(self) :
      
      
      
      
      
      
      
      def percepComputeObjectTf(self) : 
      
      def moveitIksolver(self) :
      
       










def master_node():
    rospy.wait_for_service('percepStack')
    try:
        data = 1
        while True:
            percep = rospy.ServiceProxy('percepStack', Data)

            result_1 = percep(data)
            tf = (result_1.result.x_tf, result_1.result.y_tf, result_1.result.z_tf)
            print("Result: (x_tf={}, y_tf={}, z_tf={})".format(*tf))

            #Moveit wala
            if(not data):
                rospy.wait_for_service('move_group_python_interface')
                try:
                    ik = rospy.ServiceProxy('move_group_python_interface',Target)
                    # target = Target()
                    # target.x = 0.3
                    # target.y = 0.6
                    # target.z = 0.5

                    #response = ik(0.3,0.6,0.5)
                    response = ik(*tf)

                    print(response)
                except rospy.ServiceException as e:
                    print("IK service call failed:", e)

            data = not data
            time.sleep(0.5)

    except rospy.ServiceException as e:
        print("Percep service call failed:", e)

if __name__ == "__main__":
    rospy.init_node('master_node')
    master_node()
