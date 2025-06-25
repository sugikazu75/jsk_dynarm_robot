#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import *
from std_srvs.srv import *
import time

rospy.init_node("root_fix_node")

rospy.wait_for_service("link_attacher_node/attach")
rospy.wait_for_service("gazebo/unpause_physics")

time.sleep(5.0)

attach_service = rospy.ServiceProxy("link_attacher_node/attach", Attach)
req = AttachRequest()
req.model_name_1 = "dynarm"
req.link_name_1 = "root"
req.model_name_2 = "ground_plane"
req.link_name_2 = "link"
try:
    res = attach_service(req)
except rospy.ServiceException as e:
    print("link_attacher_node/attach service call failed %s" % e)


unpause_service = rospy.ServiceProxy("gazebo/unpause_physics", Empty)
req = EmptyRequest()
try:
    res = unpause_service(req)
except rospy.ServiceException as e:
    print("gazebo/unpause_physics service call failed %s" % e)
