#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, UInt8
import time
import math


class TransformDemo:
    def __init__(self):
        self.direct_joint_angle_pub = rospy.Publisher("direct_joint_angle", JointState, queue_size=1)
        self.target_ee_pos_pub = rospy.Publisher("target_ee_final_pos", Vector3Stamped, queue_size=1)

        self.joint_names = ["joint0_roll", "joint0_pitch", "joint1_yaw", "joint1_pitch", "joint2_yaw", "joint2_pitch"]

        self.init_target_joint_q = [0, 0.5, 0, -1.0, 0, -0.5]  # initial joint angle

        self.is_transforming = 0
        is_transforming_sub = rospy.Subscriber("is_transforming", UInt8, self.isTransformingCallback)

        time.sleep(2.0)
        self.r = rospy.Rate(1)

    def isTransformingCallback(self, msg):
        self.is_transforming = msg.data

    def main(self):
        print("start main process")
        joint_state_msg = JointState()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.init_target_joint_q
        self.direct_joint_angle_pub.publish(joint_state_msg)

        last_moving_time = rospy.Time.now().to_sec()
        cnt = 0
        dt = 2.0
        split_n = 4
        center_x = 1.4
        center_y = 0.0
        center_z = 0.0
        radian = 0.6
        while not rospy.is_shutdown():
            if not self.is_transforming:
                if rospy.Time.now().to_sec() - last_moving_time > dt:
                    msg = Vector3Stamped()
                    msg.header.stamp.nsecs = int(1.5 * 1000000000)
                    msg.header.stamp.secs = 0
                    msg.vector.x = center_x
                    if cnt == 0:
                        msg.vector.y = center_y + radian
                        msg.vector.z = center_z + radian
                    elif cnt == 1:
                        msg.vector.y = center_y + radian
                        msg.vector.z = center_z - radian
                    elif cnt == 2:
                        msg.vector.y = center_y - radian
                        msg.vector.z = center_z - radian
                    elif cnt == 3:
                        msg.vector.y = center_y - radian
                        msg.vector.z = center_z + radian
                    elif cnt == 4:
                        msg.vector.y = center_y + radian
                        msg.vector.z = center_z + radian
                    self.target_ee_pos_pub.publish(msg)
                    cnt = cnt + 1
            else:
                last_moving_time = rospy.Time.now().to_sec()
            if cnt == split_n + 1:
                break
            self.r.sleep()


if __name__ == "__main__":
    rospy.init_node("dynarm_transformation_demo")
    node = TransformDemo()
    node.main()
