#! /usr/bin/env python3
import rospy                    # ROS Python API     - what does it do : it is a python client library for ROS   
import sys                      # System module      - it provides access to some variables used or maintained by the interpreter and to functions that interact strongly with the interpreter
import termios                  # Terminal module    - it provides access to the POSIX terminal control interface
import tty                      # TTY module         - it provides access to the POSIX terminal control interface
from std_msgs.msg import Int32MultiArray, String



class teleop:

    def __init__(self):        
        
        self.nodeName      = "ARM_teleop"                                                                 # Node Name
        self.joints_name   = ["base", "shoulder", "elbow", "lower_wrist", "upper_wrist", "suction"]       # Joint Names
        self.stepSize      = 1                                                                            # Step Size for each keypress


        rospy.init_node(self.nodeName, anonymous=True)                                               # Initialize the ROS Node
        self.int_array_publisher = rospy.Publisher('/int_array', Int32MultiArray, queue_size=10)
        self.array_publisher = rospy.Publisher('/jointString', String, queue_size=10)
        self.joint_values = [0, 0, 0, 0, 0, 0] 
        self.curnt_joint = 0 
        self.dir = 'cw'



    def publish_joint_states(self, joint_values):

        joint_angle_string = ', '.join(map(str, joint_values))
        print(joint_angle_string)
        int_array_msg = Int32MultiArray()
        int_array_msg.data = joint_values
        self.int_array_publisher.publish(int_array_msg)
        self.array_publisher.publish(joint_angle_string)
    


    #---------------------------------------------------------------#
    # Function Name : get_continuous_keypress()
    # Arguments     : None
    # Description   : take continuous keypress as input and move the arm to the specified pose
    #---------------------------------------------------------------#
    def get_continuous_keypress(self):


        original_terminal_settings = termios.tcgetattr(sys.stdin)       # Get the terminal settings
        
        # Print the instructions
        
        padding = " " * 27 
        print (f'{padding}\033[35;1mROBOTIC ARM CONTROLLER\033[0m')
        print("\n")
        padding = " " * 5 
        print (f'{padding}\033[33;1mTo move\033[0m')
        print ("\033[90;1mBase             :\033[0m" , "\033[1;37mpress 1\033[0m")
        print ("\033[90;1mShoulder Joint   :\033[0m" , "\033[1;37mpress 2\033[0m")
        print ("\033[90;1mElbow Joint      :\033[0m",  "\033[1;37mpress 3\033[0m")
        print ("\033[90;1mLower Wrist      :\033[0m",  "\033[1;37mpress 4\033[0m")
        print ("\033[90;1mUpper Wrist      :\033[0m",  "\033[1;37mpress 5\033[0m")
        print ("\033[90;1mSuction          :\033[0m",  "\033[1;37mpress 6\033[0m")
        print("\n\n")
        print ("\033[90;1mClock wise Rotation      :\033[0m",  "\033[1;37mpress w\033[0m")
        print ("\033[90;1mAtiClock wise Rotation   :\033[0m",  "\033[1;37mpress s\033[0m")
        print("\n\n")
        print("\033[91mPress keys (Ctrl + C + Enter to exit):\033[0m")
        print("\n")


       # take input and do the required operation
        try:
            tty.setcbreak(sys.stdin.fileno())

            while not rospy.is_shutdown():
                char = sys.stdin.read(1)
                
                if char and char != '\n':

                    
                    if char == '1'  : 
                        self.curnt_joint = 0                               # Move Base Link with keypress 1
                        self.joint = self.joints_name[self.curnt_joint]
                        print ("\033[A                                                                                  \033[A")
                        print("Joint Selected : ", self.joints_name[self.curnt_joint] , "    Direction : ", self.dir,  "    point: ", self.joint_values[self.curnt_joint])

                    if char == '2'  :   
                        self.curnt_joint = 1                              # Move Shoulder Joint with keypress 2
                        self.joint = self.joints_name[self.curnt_joint]
                        print ("\033[A                                                                                  \033[A")
                        print("Joint Selected : ", self.joints_name[self.curnt_joint] , "    Direction : ", self.dir,  "    point: ", self.joint_values[self.curnt_joint])

                
                    if char == '3'  :  
                        self.curnt_joint = 2                              # Move Elbow Joint with keypress 3
                        self.joint = self.joints_name[self.curnt_joint]
                        print ("\033[A                                                                                  \033[A")
                        print("Joint Selected : ", self.joints_name[self.curnt_joint] , "    Direction : ", self.dir,  "    point: ", self.joint_values[self.curnt_joint])


                    if char == '4'  :   
                        self.curnt_joint = 3                             # Move Lower Wrist with keypress 4
                        self.joint = self.joints_name[self.curnt_joint]
                        print ("\033[A                                                                                  \033[A")
                        print("Joint Selected : ", self.joints_name[self.curnt_joint] , "    Direction : ", self.dir,  "    point: ", self.joint_values[self.curnt_joint])


                    if char == '5'  : 
                        self.curnt_joint = 4                               # Move Upper Wrist with keypress 5
                        self.joint = self.joints_name[self.curnt_joint]
                        print ("\033[A                                                                                  \033[A")
                        print("Joint Selected : ", self.joints_name[self.curnt_joint] , "    Direction : ", self.dir,  "    point: ", self.joint_values[self.curnt_joint])


                    if char == '6'  :  
                        self.curnt_joint = 5                              # Move Upper Wrist with keypress 5
                        self.joint = self.joints_name[self.curnt_joint]
                        print ("\033[A                                                                                  \033[A")
                        print("Joint Selected : ", self.joints_name[self.curnt_joint] , "    Direction : ", self.dir,  "    point: ", self.joint_values[self.curnt_joint])

                    if char == 'w'  :                                     # Move Clockwise with keypress w
                        self.dir = 'cw'
                        print ("\033[A                                                                                  \033[A")
                        self.joint_values[self.curnt_joint] = 1
                        print("Joint Selected : ", self.joints_name[self.curnt_joint] , "    Direction : ", self.dir,  "    point: ", self.joint_values[self.curnt_joint])
                        self.publish_joint_states(self.joint_values)


                    if char == 's'  :                                # Move AntiClockwise with keypress s
                        self.dir = 'ccw'
                        print ("\033[A                                                                                  \033[A")
                        self.joint_values[self.curnt_joint] = 2
                        print("Joint Selected : ", self.joints_name[self.curnt_joint] , "    Direction : ", self.dir,  "    point: ", self.joint_values[self.curnt_joint])
                        self.publish_joint_states(self.joint_values)

                    if char == 'h'  :                                # Move AntiClockwise with keypress s
                        self.dir = 'ccw'
                        print ("\033[A                                                                                  \033[A")
                        self.joint_values[self.curnt_joint] = 0
                        print("Joint Selected : ", self.joints_name[self.curnt_joint] , "    Direction : ", self.dir,  "    point: ", self.joint_values[self.curnt_joint])
                        self.publish_joint_states(self.joint_values)



                    
        except KeyboardInterrupt:
            pass
        finally:
            print("Exiting...")
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_terminal_settings)


if __name__ == "__main__":
    teleopObj = teleop()
    teleopObj.get_continuous_keypress()
