#! /usr/bin/env python3
import rospy                    # ROS Python API     - what does it do : it is a python client library for ROS   
import sys                      # System module      - it provides access to some variables used or maintained by the interpreter and to functions that interact strongly with the interpreter
import moveit_commander         # MoveIt Commander   - it is a ROS wrapper that provides many of the ROS wrapper that provides many of the ROS MoveIt functions in an easy-to-use API
import moveit_msgs.msg          # MoveIt Messages    - it is a ROS wrapper that provides many of the ROS MoveIt functions in an easy-to-use API
import actionlib                # Action Library     - it provides a standardized interface for interfacing with preemptable tasks
import math                     # Math module        - it provides access to the mathematical functions defined by the C standard
import json                     # JSON module        - it is a lightweight data interchange format inspired by JavaScript object literal syntax
import enquiries                # Enquiries module   - it is a lightweight data interchange format inspired by JavaScript object literal syntax
import termios                  # Terminal module    - it provides access to the POSIX terminal control interface
import tty                      # TTY module         - it provides access to the POSIX terminal control interface


# Global Variables
nodeName      = "PicknPlace_Node"    # Node Name
armGroup      = "arm_gp"             # Planning Group
gripperGroup  = "gripper_gp"         # Gripper Group
stepSize      = 0.4                  # Step Size

# Joint Angle Limits
dicAngle      = { "base_joint" : [-3.14, 3.14], "shoulder_joint" : [-1.57 , 1.57], "arm_joint" : [-2.87, 2.87], "forearm_joint" : [-2.87, 2.87], "wrist_joint" : [-2.87, 2.87] }


class AjgarMoveit:

    def __init__(self):
        
        rospy.init_node(nodeName, anonymous=True)                                               # Initialize the ROS Node
        self.arm_planning_group = armGroup                                                      # Planning Group
        self.gripper_planning_group = gripperGroup                                              # Gripper Group
        self._commander = moveit_commander.roscpp_initialize(sys.argv)                          # Initialize the ROS API
        self._robot = moveit_commander.RobotCommander()                                         # Initialize the RobotCommander
        self._scene = moveit_commander.PlanningSceneInterface()                                 # Initialize the PlanningSceneInterface
        self.arm_group = moveit_commander.MoveGroupCommander(self.arm_planning_group)           # Initialize the MoveGroupCommander
        self.gripper_group = moveit_commander.MoveGroupCommander(self.gripper_planning_group)   # Initialize the MoveGroupCommander

        # Display Trajectory in RViz
        self._display_trajectory_publisher = rospy.Publisher( "/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=1 )

        # A ROS Publisher for the UR5 Joint Angles
        self._exectute_trajectory_client = actionlib.SimpleActionClient("execute_trajectory", moveit_msgs.msg.ExecuteTrajectoryAction)

        # Wait for the ROS Action Server to come up 
        self._exectute_trajectory_client.wait_for_server()
        self._planning_frame = self.arm_group.get_planning_frame()
        self._eef_link = self.arm_group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        # Print the ROS Parameters
        rospy.loginfo( "Planning Group: "    + "\033[94m" + "{}".format(self._planning_frame) + "\033[0m")
        rospy.loginfo( "End Effector Link: " + "\033[94m" + "{}".format(self._eef_link) + "\033[0m")
        rospy.loginfo( "Group Names:"        + "\033[94m" + "{}".format(self._group_names) + "\033[0m")
        rospy.loginfo("\033[94m" + "Ur5Moveit init done." + "\033[0m")



      #---------------------------------------------------------------# 
      # Function Name : set_joint_angles()
      # Arguments     : arg_list_joint_angles - List of Joint Angles
      # Description   : take list of joint angles as input and move the arm to the specified joint angles
      #---------------------------------------------------------------#
    def set_joint_angles(self, arg_list_joint_angles):
    
                                                                            
        list_joint_values = self.arm_group.get_current_joint_values()             # Get the current joint values 
        rospy.loginfo("\033[94m" + ">>> Current Joint Values:" + "\033[0m")       # Print the current joint values
        rospy.loginfo(list_joint_values)                                          # Print the current joint values

        self.arm_group.set_joint_value_target(arg_list_joint_angles)              # Set the joint angles for the arm group
        self.arm_group.plan()                                                     # Plan the trajectory
        flag_plan = self.arm_group.go(wait=True)                                  # Execute the trajectory and wait for the execution to finish

        list_joint_values = self.arm_group.get_current_joint_values()             # Get the current joint values
        rospy.loginfo("\033[94m" + ">>> Final Joint Values:" + "\033[0m")         # Print the final joint values
        rospy.loginfo(list_joint_values)                                          # Print the final joint values

        pose_values = self.arm_group.get_current_pose().pose                      # Get the current pose values
        rospy.loginfo("\033[94m" + ">>> Final Pose:" + "\033[0m")                 # Print the final pose values
        rospy.loginfo(pose_values)                                                # Print the final pose values

        if flag_plan == True:                          
            rospy.loginfo("\033[94m" + ">>> set_joint_angles() Success" + "\033[0m")



    #---------------------------------------------------------------#
    # Function Name : set_pose()
    # Arguments     : arg_list_joint_angles - List of Joint Angles of gripper 
    # Description   : take pose as input and move the arm to the specified pose
    #---------------------------------------------------------------#
        
    def gripper_joint_angles(self, arg_list_joint_angles):
    
        list_joint_values = self.gripper_group.get_current_joint_values()               # Get the current joint values
        rospy.loginfo("\033[94m" + ">>> Current Joint Values:" + "\033[0m")             # Print the current joint values
        rospy.loginfo(list_joint_values)                                                # Print the current joint values
      
        self.gripper_group.set_joint_value_target(arg_list_joint_angles)                # Set the joint angles for the gripper group
        self.gripper_group.plan()                                                       # Plan the trajectory
        flag_plan = self.gripper_group.go(wait=True)                                    # Execute the trajectory and wait for the execution to finish

        if flag_plan == True:                                                          
            rospy.loginfo("\033[94m" + ">>> set_joint_angles() Success" + "\033[0m")




    #---------------------------------------------------------------#
    # Function Name : set_pose()
    # Arguments     : joint_name - Name of the joint, index - index of the joint in the list, sign - 1 for positive and 0 for negative direction
    # Description   : take pose as input and move the arm to the specified pose
    #---------------------------------------------------------------#

    def keypress_jointAngle(self, joint_name, index, sign) :


        list_joint_values = self.arm_group.get_current_joint_values()            # Get the current joint values
        joint_angle = round(list_joint_values[index], 2)                         # Get the current joint angle of the joint 

        if sign == 1 :                                                           # If sign is 1 then increase the joint angle by stepSize
            joint_angle = joint_angle + stepSize                                 
        elif sign == 0 :                                                         # If sign is 0 then decrease the joint angle by stepSize
            joint_angle = joint_angle - stepSize 


        # Check if the joint angle is within the joint limits
        if (dicAngle.get(joint_name)[0] < joint_angle and dicAngle.get(joint_name)[1] > joint_angle ) :
            
            self.arm_group.set_joint_value_target(joint_name, joint_angle)             # Set the joint angles for the arm group
            self.arm_group.plan()                                                      # Plan the trajectory
            flag_plan = self.arm_group.go()                                            # Execute the trajectory and wait for the execution to finish
            list_joint_values = self.arm_group.get_current_joint_values()              # Get the current joint values 
            pose_values = self.arm_group.get_current_pose().pose                       # Get the current pose values
            


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
        print ("\033[90;1mBase Link :\033[0m" , "\033[1;37mpress x / c\033[0m")
        print ("\033[90;1mJoint I   :\033[0m" , "\033[1;37mpress w / s\033[0m")
        print ("\033[90;1mJoint II  :\033[0m",  "\033[1;37mpress e / d\033[0m")
        print ("\033[90;1mJoint III :\033[0m",  "\033[1;37mpress r / f\033[0m")
        print ("\033[90;1mJoint IV  :\033[0m",  "\033[1;37mpress g / h\033[0m")
        print("\n\n")
        print("\033[91mPress keys (Ctrl + C + Enter to exit):\033[0m")

       # take input and do the required operation
        try:
            tty.setcbreak(sys.stdin.fileno())

            while not rospy.is_shutdown():
                char = sys.stdin.read(1)
                if char and char != '\n':
                    
                                        
                  
                    if char == 'x' or char == 'c' :                                # Move Base Link with keypress x / c
                        if char == 'x' :
                            sign = 1
                        else :
                            sign = 0 
                        self.keypress_jointAngle("joint_1", 0, sign)

                    
                    if char == 'g' or char == 'h' :                                # Move Joint_V with keypress w / s
                        if char == 'g' :
                            sign = 1
                        else :
                            sign = 0 
                        self.keypress_jointAngle("joint_5", 4, sign)

                    
                    if char == 'w' or char == 's' :                               # Move Joint_I with keypress w / s
                        if char == 'w' :
                            sign = 1
                        else :
                            sign = 0 
                        self.keypress_jointAngle("joint_2", 1, sign)

                   
                    if char == 'e' or char == 'd' :                                # Move Joint_II with keypress e / d
                        if char == 'e' :
                            sign = 1
                        else :
                            sign = 0 
                        self.keypress_jointAngle("joint_3", 2, sign)

                   
                    if char == 'r' or char == 'f' :                                 # Move Joint_III with keypress r / f
                        if char == 'r' :
                            sign = 1
                        else :
                            sign = 0 
                        self.keypress_jointAngle("joint_4", 3, sign)

        except KeyboardInterrupt:
            pass
        finally:
            print("Exiting...")
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_terminal_settings)



    #---------------------------------------------------------------#
    # Function Name : _del_()
    # Arguments     : None
    # Description   : Destructor to delete the object of class Ur5Moveit
    #---------------------------------------------------------------#
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo("\033[94m" + "Object of class Ur5Moveit Deleted." + "\033[0m")



#---------------------------------------------------------------#
# Function Name : print_colored()
# Arguments     : text- text to be printed , color_code - color code for the text
# Description   : print the text in the specified color
#---------------------------------------------------------------#
def print_colored(text, color_code):
    print(f"\033[{color_code}m{text}\033[0m")




#---------------------------------------------------------------#
# Function Name : main()
# Arguments     : None
# Description   : main function - entry point of the program
#---------------------------------------------------------------#
def main():

    # Initialize the importaant details to be printed on the terminal
    text = "FSM - IITD-AIA Foundation For Smart Manufacturing"
    width = 79
    middle = math.floor(width / 2)
    padding = middle - len(text) // 2
    print(
        f"{' ' * padding}\033[1m\033[37m\033[40m\033[18m{text}\033[0m{' ' * padding}\n"
    )
    options = [
        "1. Automatic Pick and Place",
        "2. Manual Controller using KeyBoard",
        "3. About Project and Dependencies",
    ]
    choice = enquiries.choose("Choose one of these options: \n", options)


    # Call the required function based on the choice
    if choice == "1. Automatic Pick and Place":

        print(choice[3:])
        
        ur5 = Ur5Moveit()                                                             # Create an object of class Ur5Moveit
           
        with open(
            "/home/jc/catkin_ws/src/robot_arm_urdf/scripts/angle_data.json", "r"
        ) as f:
            data = json.load(f)


        # Get the required data from the json file
        groupTypeList = ["JOINTS_Positions", "Gripper"]
        gripperStateList = ["OPEN", "CLOSE"]
        packetTypeList = ["Green_Yellow_Packets", "Blue_Red_Packets"]
        actionPerformList = ["PICK", "INBW", "PLACE"]
        angleNeededList = []

       
        while not rospy.is_shutdown():
            for packetType in packetTypeList:
                for actionPerform in actionPerformList:
                    for angleIndex in range(5):
                        angleRadian = math.radians(
                            data[groupTypeList[0]][packetType][actionPerform][
                                "Angle_Needed"
                            ][angleIndex]
                        )
                        angleNeededList.append(angleRadian)
                    ur5.set_joint_angles(angleNeededList)
                    angleNeededList = []

                    if actionPerform == "PICK":
                        for angleIndex in range(8):
                            angleRadian = math.radians(
                                data[groupTypeList[1]][gripperStateList[1]][
                                    "Angle_Needed"
                                ][angleIndex]
                            )
                            angleNeededList.append(angleRadian)
                        ur5.gripper_joint_angles(angleNeededList)
                        angleNeededList = []
                        rospy.sleep(2)

                    elif actionPerform == "PLACE":
                        for angleIndex in range(8):
                            angleRadian = math.radians(
                                data[groupTypeList[1]][gripperStateList[0]][
                                    "Angle_Needed"
                                ][angleIndex]
                            )
                            angleNeededList.append(angleRadian)
                        ur5.gripper_joint_angles(angleNeededList)
                        angleNeededList = []

            print("Task Completed")
            break


    elif choice == "3. About Project and Dependencies":

        print("About: ")
        print_colored("Pick and Place using UR5 Robotic ARM", "94")  # Blue
        print("Created By: ")
        print_colored("JAYESH CHAUDHARY / jayeshchaudhary21jan@gmail.com", "94")  # Blue
        print("")
        print_colored("This project is an internship project created by Jayesh Chaudhary during an internship at FSM - IITD-AIA Foundation For Smart Manufacturing under the guidance of KRRISH JINDAL.","97",)  
        print("")
        print_colored("Description:", "92")  
        print("    To run this project, the following tech stack is needed:")
        print_colored("        -> ROS Noetic", "92")  
        print_colored("        -> Moveit", "92")  
        print_colored("        -> Python 3.8", "92")  
        print_colored("        -> Ubuntu 20.04", "92")  
        print("")
        print_colored("This project is created using UR5 Robotic Arm URDF and Moveit. It uses Moveit to plan the path and then execute the path and python script to call the Moveit API and to read the JSON file which contains the angles needed to be set for the given task.\n","97") 


    elif choice == "2. Manual Controller using KeyBoard" :

        ur5 = Ur5Moveit()
        ur5.get_continuous_keypress()


if __name__ == "__main__":
    main()