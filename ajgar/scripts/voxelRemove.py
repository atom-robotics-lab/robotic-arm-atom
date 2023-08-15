#!/usr/bin/env python3
import moveit_commander
import sys
import rospy
from octomap_msgs.msg import Octomap
from octomap_msgs.msg import FullMap

def octomap_callback(octomap_msg):
    # Modify the octomap by marking voxels within the box as free
    modified_octomap = octomap_msg
    for x in range(min_voxel_coords[0], max_voxel_coords[0] + 1):
        for y in range(min_voxel_coords[1], max_voxel_coords[1] + 1):
            for z in range(min_voxel_coords[2], max_voxel_coords[2] + 1):
                # Modify the octomap voxel data
                # ...

    # Publish the modified octomap
    modified_octomap_pub.publish(modified_octomap)

def modify_octomap():
    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Connect to the MoveIt planning scene
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Define the coordinates of the square box in voxel space
    min_voxel_coords = (1, 2, 3)
    max_voxel_coords = (2, 3, 4)

    # Create a ROS node
    rospy.init_node("modify_octomap_node")

    # Subscribe to the original octomap topic
    original_octomap_sub = rospy.Subscriber('/your/original/octomap/topic', Octomap, octomap_callback)

    # Create a publisher for the modified octomap
    modified_octomap_pub = rospy.Publisher('/your/modified/octomap/topic', Octomap, queue_size=10)

    # Spin to keep the node running
    rospy.spin()

    # Clean up
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    modify_octomap()

