#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState
import math
import time


class ResetJointAngles:
    def __init__(self):
        self.joint_command_pub = rospy.Publisher("joint_command", JointState, queue_size=1)
        self.joints_control_pub = rospy.Publisher("joints_ctrl", JointState, queue_size=1)
        self.joint_names = ["joint1_yaw", "joint1_pitch", "joint2_yaw", "joint2_pitch", "joint3_yaw", "joint3_pitch"]
        self.reset_joint_angles = [math.pi / 2.0, 0.0, math.pi / 2.0, 0.0, math.pi / 2.0, 0.0]
        time.sleep(2.0)

    def main(self):
        rospy.loginfo("Sending reset joint angles")
        joint_state_msg = JointState()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.reset_joint_angles
        self.joint_command_pub.publish(joint_state_msg)
        self.joints_control_pub.publish(joint_state_msg)


if __name__ == "__main__":
    rospy.init_node("reset_joint_angles")
    node = ResetJointAngles()
    node.main()
