#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import time


class CircleTrajectoryDemo:
    def __init__(self):
        self.direct_joint_angle_pub = rospy.Publisher("direct_joint_angle", JointState, queue_size=1)
        self.circle_trajectory_pub = rospy.Publisher("circle_trajectory", Float32MultiArray, queue_size=1)

        self.joint_names = ["joint0_roll", "joint0_pitch", "joint1_yaw", "joint1_pitch", "joint2_yaw", "joint2_pitch"]
        self.joint_angle = [0, 1.0, 0, -1.0, 0, -1.0]

        self.radius = 0.3
        self.angvel = 1.5

        time.sleep(2.0)

    def main(self):
        print("start main process")
        joint_state_msg = JointState()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.joint_angle
        self.direct_joint_angle_pub.publish(joint_state_msg)

        time.sleep(5.0)

        circle_trajectory_msg = Float32MultiArray()
        circle_trajectory_msg.data.append(self.radius)
        circle_trajectory_msg.data.append(self.angvel)
        self.circle_trajectory_pub.publish(circle_trajectory_msg)


if __name__ == "__main__":
    rospy.init_node("circle_trajectory_demo")
    node = CircleTrajectoryDemo()
    node.main()
