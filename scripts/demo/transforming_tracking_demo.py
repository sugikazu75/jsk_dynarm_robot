#!/usr/bin/env python

import rospy

from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import Vector3, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float32MultiArray
import math
import time


class TransformingTrackingDemo:
    def __init__(self):
        self.transforming_tracking_pub = rospy.Publisher("transforming_tracking_command", Empty, queue_size=1)
        self.uav_nav_pub = rospy.Publisher("uav/nav", FlightNav, queue_size=1)
        self.ddp_root_pose_pub = rospy.Publisher("root_pose_command", Pose, queue_size=1)
        self.joint_command_pub = rospy.Publisher("joint_command", JointState, queue_size=1)
        self.joints_control_pub = rospy.Publisher("joints_ctrl", JointState, queue_size=1)

        self.iniital_pos = [0.0, 0.0, 1.0]

        circcle_radius = 0.5
        circle_duration = 3.0
        circle_loop = 4

        joint_duration = 2.0
        joint_loop = 6
        joint_names = ["joint1_yaw", "joint1_pitch", "joint2_yaw", "joint2_pitch", "joint3_yaw", "joint3_pitch"]
        start_angles = [math.pi / 2.0, 0.0, math.pi / 2.0, 0.0, math.pi / 2.0, 0.0]
        end_angles = [1.0, -0.5, 1.0, 0.5, 1.0, -0.5]

        rospy.set_param("circle_trajectory/radius", circcle_radius)
        rospy.set_param("circle_trajectory/duration", circle_duration)
        rospy.set_param("circle_trajectory/loop", circle_loop)

        rospy.set_param("joint_trajectory/duration", joint_duration)
        rospy.set_param("joint_trajectory/loop", joint_loop)
        rospy.set_param("joint_trajectory/joint_names", joint_names)
        rospy.set_param("joint_trajectory/start_angle", start_angles)
        rospy.set_param("joint_trajectory/end_angle", end_angles)

        rospy.loginfo("set parameters")

        time.sleep(2.0)

    def main(self):
        uav_nav_msg = FlightNav()
        uav_nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        uav_nav_msg.target_pos_x = self.iniital_pos[0]
        uav_nav_msg.target_pos_y = self.iniital_pos[1]
        uav_nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
        uav_nav_msg.target_pos_z = self.iniital_pos[2]
        uav_nav_msg.yaw_nav_mode = FlightNav.POS_MODE
        uav_nav_msg.target_yaw = 0.0
        self.uav_nav_pub.publish(uav_nav_msg)

        root_pose_msg = Pose()
        root_pose_msg.position.x = self.iniital_pos[0]
        root_pose_msg.position.y = self.iniital_pos[1]
        root_pose_msg.position.z = self.iniital_pos[2]
        root_pose_msg.orientation.x = 0.0
        root_pose_msg.orientation.y = 0.0
        root_pose_msg.orientation.z = 0.0
        root_pose_msg.orientation.w = 1.0
        self.ddp_root_pose_pub.publish(root_pose_msg)
        rospy.sleep(5.0)

        self.transforming_tracking_pub.publish(Empty())


if __name__ == "__main__":
    rospy.init_node("transforming_tracking_demo")
    node = TransformingTrackingDemo()
    node.main()
