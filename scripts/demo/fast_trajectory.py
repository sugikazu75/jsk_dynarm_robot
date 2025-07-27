#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3Stamped
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import time


class FastLinearTrajectoryDemo:
    def __init__(self):
        self.direct_joint_angle_pub = rospy.Publisher("direct_joint_angle", JointState, queue_size=1)
        self.target_ee_pos_pub = rospy.Publisher("target_ee_final_pos", Vector3Stamped, queue_size=1)

        self.joint_names = ["joint0_roll", "joint0_pitch", "joint1_yaw", "joint1_pitch", "joint2_yaw", "joint2_pitch"]

        self.init_target_joint_q = [0, 1.0, 0, -1.0, 0, 0]
        self.init_target_ee_pos = [1.5, 0, -0.5]
        self.final_target_ee_pos = [1.5, 0, 0.8]  # final end-effector position

        time.sleep(2.0)

    def main(self):
        print("start main process")
        joint_state_msg = JointState()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.init_target_joint_q
        self.direct_joint_angle_pub.publish(joint_state_msg)

        time.sleep(2.0)

        init_target_ee_pos_msg = Vector3Stamped()
        init_target_ee_pos_msg.header.stamp.nsecs = int(2 * 1000000000)
        init_target_ee_pos_msg.header.stamp.secs = 0
        init_target_ee_pos_msg.vector.x = self.init_target_ee_pos[0]
        init_target_ee_pos_msg.vector.y = self.init_target_ee_pos[1]
        init_target_ee_pos_msg.vector.z = self.init_target_ee_pos[2]
        self.target_ee_pos_pub.publish(init_target_ee_pos_msg)

        time.sleep(4.0)

        final_target_ee_pos_msg = Vector3Stamped()
        final_target_ee_pos_msg.header.stamp.nsecs = int(1.0 * 1000000000)
        final_target_ee_pos_msg.header.stamp.secs = 0
        final_target_ee_pos_msg.vector.x = self.final_target_ee_pos[0]
        final_target_ee_pos_msg.vector.y = self.final_target_ee_pos[1]
        final_target_ee_pos_msg.vector.z = self.final_target_ee_pos[2]
        self.target_ee_pos_pub.publish(final_target_ee_pos_msg)


if __name__ == "__main__":
    rospy.init_node("circle_trajectory_demo")
    node = FastLinearTrajectoryDemo()
    node.main()
