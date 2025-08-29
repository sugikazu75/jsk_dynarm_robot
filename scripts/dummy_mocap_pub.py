#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped


class DummyMocapPub:
    def __init__(self):
        rospy.init_node("dummy_mocap_pub_node")
        self.pub = rospy.Publisher("mocap/pose", PoseStamped, queue_size=10)
        self.rate = rospy.Rate(100)  # 100 Hz

    def run(self):
        while not rospy.is_shutdown():
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = 0.0
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = 1.0
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0

            self.pub.publish(pose_msg)
            self.rate.sleep()


if __name__ == "__main__":
    dummy_mocap_pub = DummyMocapPub()
    dummy_mocap_pub.run()
