#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState
import math
import time


class TransformDemo:
    def __init__(self, mode):
        self.joint_command_pub = None
        if mode == 0:  # for ddp
            self.joint_command_pub = rospy.Publisher("joint_command", JointState, queue_size=1)
        elif mode == 1:  # for full vectoring
            self.joint_command_pub = rospy.Publisher("joints_ctrl", JointState, queue_size=1)

        self.joint_names = ["joint1_yaw", "joint1_pitch", "joint2_yaw", "joint2_pitch", "joint3_yaw", "joint3_pitch"]
        self.initial_joint_angle = [math.pi / 2.0, 0.0, math.pi / 2.0, 0.0, math.pi / 2.0, 0.0]
        self.target_joint_angle = [math.pi / 2.0, 0.0, 0.0, 0.0, -math.pi / 2.0, 0.0]

        rospy.loginfo("mode: {}".format(mode))

        time.sleep(2.0)

    def main(self):
        rospy.loginfo("start main process")
        rospy.loginfo("send initial joint angle")
        joint_state_msg = JointState()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.initial_joint_angle
        self.joint_command_pub.publish(joint_state_msg)

        time.sleep(5.0)
        rospy.loginfo("send transformed joint angle")
        joint_state_msg.position = self.target_joint_angle
        self.joint_command_pub.publish(joint_state_msg)


if __name__ == "__main__":
    rospy.init_node("transform_demo")
    mode = rospy.get_param("~mode", 0)
    node = TransformDemo(mode)
    node.main()
