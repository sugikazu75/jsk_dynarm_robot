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

        self.init_target_joint_q = [0, 0.8, 0, -1.0, 0, -0.5]  # initial joint angle
        self.init_target_ee_pos = [1.6, 0.0, 0.0]

        self.is_transforming = 0
        is_transforming_sub = rospy.Subscriber("debug/is_transforming", UInt8, self.isTransformingCallback)

        time.sleep(2.0)
        self.r = rospy.Rate(1)

    def isTransformingCallback(self, msg):
        self.is_transforming = msg.data

    def main(self):
        print("start main process")
        joint_state_msg = JointState()
        joint_state_msg.header.stamp.secs = 5
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.init_target_joint_q
        self.direct_joint_angle_pub.publish(joint_state_msg)
        rospy.loginfo("move to initial joint angle %s" % self.init_target_joint_q)
        time.sleep(6.0)

        target_ee_pos_msg = Vector3Stamped()
        target_ee_pos_msg.header.stamp.secs = 2
        target_ee_pos_msg.vector.x = self.init_target_ee_pos[0]
        target_ee_pos_msg.vector.y = self.init_target_ee_pos[1]
        target_ee_pos_msg.vector.z = self.init_target_ee_pos[2]
        self.target_ee_pos_pub.publish(target_ee_pos_msg)
        rospy.loginfo("move to initial position %s" % self.init_target_ee_pos)

        last_moving_time = rospy.Time.now().to_sec()
        cnt = 0
        dt = 0.5
        range_x = 0.2
        range_y = 0.3
        range_z = 0.3
        max_x = self.init_target_ee_pos[0] + range_x
        min_x = self.init_target_ee_pos[0] - range_x
        center_x = [max_x, max_x, min_x, min_x, max_x, max_x, min_x, min_x, self.init_target_ee_pos[0]]
        center_y = [
            range_y,
            -range_y,
            range_y,
            -range_y,
            range_y,
            -range_y,
            range_y,
            -range_y,
            self.init_target_ee_pos[1],
        ]
        center_z = [
            range_z,
            range_z,
            -range_z,
            -range_z,
            -range_z,
            -range_z,
            range_z,
            range_z,
            self.init_target_ee_pos[2],
        ]
        while not rospy.is_shutdown():
            if not self.is_transforming:
                if rospy.Time.now().to_sec() - last_moving_time > dt:
                    msg = Vector3Stamped()
                    msg.header.stamp.nsecs = int(1.0 * 1000000000)
                    msg.header.stamp.secs = 0
                    msg.vector.x = center_x[cnt]
                    msg.vector.y = center_y[cnt]
                    msg.vector.z = center_z[cnt]
                    self.target_ee_pos_pub.publish(msg)
                    cnt = cnt + 1
                    rospy.loginfo("move to %s" % [msg.vector.x, msg.vector.y, msg.vector.z])
            else:
                last_moving_time = rospy.Time.now().to_sec()
            if cnt == len(center_x):
                break
            self.r.sleep()


if __name__ == "__main__":
    rospy.init_node("dynarm_transformation_demo")
    node = TransformDemo()
    node.main()
