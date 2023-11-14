#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import math
from trac_ik_python.trac_ik import IK 

class MoveGroupPythonInterface(object):
    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous=True)
        group_name = "arm_group"
        group = moveit_commander.MoveGroupCommander(group_name)
        group.set_start_state_to_current_state()
        robot = moveit_commander.RobotCommander()
        self.group = group

    def go_to_joint_state(self):
        group = self.group

        urdfstring = ''.join(open('arm.urdf', 'r').readlines())
        ik = IK('base_link', 'end', urdf_string=urdfstring)
        pi = 22 / 7
        x_pos = 0.2
        y_pos = 0.2
        z_pos = 0.5
        self.angleTuple = ik.get_ik([0.0] * 6, x_pos, y_pos, z_pos, 0, 0, 0, 1)
        joint = ['Base Joint       ',
                 'Shoulder Joint   ',
                 'Elbow Joint      ',
                 'Lower Wrist Joint',
                 'Upper Wrist Joint',
                 'Suction          ']

        print(x_pos**2 + y_pos**2 + z_pos**2)
        if self.angleTuple is not None:
            for angle in range(0, 6):
                print(joint[angle], " ", str(self.angleTuple[angle] * (180/pi)))
            print(" ")
        else:
            print(self.angleTuple)

        print("rosrun tf static_transform_publisher", x_pos, y_pos, z_pos, "0 1 0 base_link rnd_tf 100")

        joint_goal = group.get_current_joint_values()
        joint_goal[0] = math.radians(float(self.angleTuple[0] * (180/pi)))
        joint_goal[1] = math.radians(float(self.angleTuple[1] * (180/pi)))
        joint_goal[2] = math.radians(float(self.angleTuple[2] * (180/pi)))
        joint_goal[3] = math.radians(float(self.angleTuple[3] * (180/pi)))
        joint_goal[4] = math.radians(float(self.angleTuple[4] * (180/pi)))

        group.go(joint_goal, wait=True)
        group.stop()

def main():
    interface = MoveGroupPythonInterface()
    interface.go_to_joint_state()

if __name__ == '__main__':
    main()
