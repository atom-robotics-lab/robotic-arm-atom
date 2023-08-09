#!/usr/bin/env python3

import rospy
from plugin_pneumatic_gripper.srv import Attach, AttachRequest, AttachResponse


if __name__ == '__main__':
    rospy.init_node('demo_detach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

    # Link them
    rospy.loginfo("Detaching cube1 and cube2")
    req = AttachRequest()
    req.model_name_1 = "ajgar"
    req.link_name_1 = "suction_1"
    req.model_name_2 = "medium_box_1_clone_2"
    req.link_name_2 = "link"

    attach_srv.call(req)
    