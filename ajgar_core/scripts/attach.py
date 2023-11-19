#!/usr/bin/env python3

import rospy
from plugin_pneumatic_gripper.srv import Attach, AttachRequest, AttachResponse

def attach_links(model):
    #rospy.init_node('demo_attach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    rospy.loginfo("Attaching bodies and bodies")
    req = AttachRequest()
    req.model_name_1 = "ajgar"
    req.link_name_1 = "suction"
    req.model_name_2 = model
    req.link_name_2 = "link"

    attach_srv.call(req)
    
    rospy.loginfo("Links attached")
