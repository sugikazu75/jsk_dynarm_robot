#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import time

rospy.init_node("gimbal_test")

gimbal_control_pub = rospy.Publisher("dynarm/gimbals_ctrl", JointState, queue_size=10)
radian = rospy.get_param("~radian", 0)
joint_state_msg = JointState()
joint_state_msg.name = [
    "gimbal1_roll",
    "gimbal1_pitch",
    "gimbal2_roll",
    "gimbal2_pitch",
    "gimbal3_roll",
    "gimbal3_pitch",
]
joint_state_msg.position = [radian, radian, radian, radian, radian, radian]

time.sleep(0.6)
gimbal_control_pub.publish(joint_state_msg)
