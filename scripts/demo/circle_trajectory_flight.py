#!/usr/bin/env python

import rospy

from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import Vector3, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float32MultiArray
import math
import time


class CircleTrajectoryFlightDemo:
    def __init__(self):
        self.circle_trajectory_pub = rospy.Publisher("circle_trajectory_command", Empty, queue_size=1)
        self.uav_nav_pub = rospy.Publisher("uav/nav", FlightNav, queue_size=1)
        self.ddp_root_pose_pub = rospy.Publisher("root_pose_command", Pose, queue_size=1)
        self.joint_command_pub = rospy.Publisher("joint_command", JointState, queue_size=1)
        self.joints_control_pub = rospy.Publisher("joints_ctrl", JointState, queue_size=1)

        radius = 0.5
        duration = 3.0
        loop = 4

        rospy.set_param("circle_trajectory/radius", radius)
        rospy.set_param("circle_trajectory/duration", duration)
        rospy.set_param("circle_trajectory/loop", loop)

        rospy.loginfo("set circle trajectory parameters")

        rospy.sleep(2.0)

    def main(self):
        rospy.loginfo("start main process")
        rospy.loginfo("send initial joint angle")
        joint_comnmand_msg = JointState()
        joint_comnmand_msg.name = [
            "joint1_yaw",
            "joint1_pitch",
            "joint2_yaw",
            "joint2_pitch",
            "joint3_yaw",
            "joint3_pitch",
        ]
        joint_comnmand_msg.position = [math.pi / 2.0, 0.0, math.pi / 2.0, 0.0, math.pi / 2.0, 0.0]
        self.joint_command_pub.publish(joint_comnmand_msg)
        self.joints_control_pub.publish(joint_comnmand_msg)
        time.sleep(5.0)

        rospy.loginfo("move to start position")
        uav_nav_msg = FlightNav()
        uav_nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        uav_nav_msg.target_pos_x = 0.0
        uav_nav_msg.target_pos_y = 0.0
        uav_nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
        uav_nav_msg.target_pos_z = 1.0
        uav_nav_msg.yaw_nav_mode = FlightNav.POS_MODE
        uav_nav_msg.target_yaw = 0.0
        self.uav_nav_pub.publish(uav_nav_msg)

        root_pose_msg = Pose()
        root_pose_msg.position.x = 0.0
        root_pose_msg.position.y = 0.0
        root_pose_msg.position.z = 1.0
        root_pose_msg.orientation.x = 0.0
        root_pose_msg.orientation.y = 0.0
        root_pose_msg.orientation.z = 0.0
        root_pose_msg.orientation.w = 1.0
        self.ddp_root_pose_pub.publish(root_pose_msg)
        time.sleep(10.0)

        rospy.loginfo("send circle trajectory")
        circle_trajectory_msg = Empty()
        self.circle_trajectory_pub.publish(circle_trajectory_msg)


if __name__ == "__main__":
    rospy.init_node("circle_trajectory_flight_demo")
    node = CircleTrajectoryFlightDemo()
    node.main()
