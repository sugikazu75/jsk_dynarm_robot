#!/usr/bin/env python

import rospy

from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import Vector3, Pose, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
import math
import time


class LinearMotionFlightDemo:
    def __init__(self):
        self.uav_nav_pub = rospy.Publisher("uav/nav", FlightNav, queue_size=1)
        self.ddp_root_pose_pub = rospy.Publisher("root_pose_command", Pose, queue_size=1)
        self.joint_command_pub = rospy.Publisher("joint_command", JointState, queue_size=1)
        self.joints_control_pub = rospy.Publisher("joints_ctrl", JointState, queue_size=1)
        self.trajectory_path_pub = rospy.Publisher("trajectory_path", Path, queue_size=1)

        self.iniital_pos = [0.0, 0.0, 1.0]
        self.target_pos = [1.0, 0.0, 1.0]
        rospy.sleep(2.0)

    def main(self):
        rospy.loginfo("start main process")
        rospy.loginfo("send initial joint angle")
        joint_command_msg = JointState()
        joint_command_msg.name = [
            "joint1_yaw",
            "joint1_pitch",
            "joint2_yaw",
            "joint2_pitch",
            "joint3_yaw",
            "joint3_pitch",
        ]
        joint_command_msg.position = [math.pi / 2.0, 0.0, math.pi / 2.0, 0.0, math.pi / 2.0, 0.0]
        self.joint_command_pub.publish(joint_command_msg)
        self.joints_control_pub.publish(joint_command_msg)
        time.sleep(5.0)

        rospy.loginfo("move to start position")
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

        rospy.loginfo("move to target position")
        path_msg = Path()
        path_msg.header.frame_id = "world"
        path_msg.header.stamp = rospy.Time.now()
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = self.iniital_pos[0]
        pose_msg.pose.position.y = self.iniital_pos[1]
        pose_msg.pose.position.z = self.iniital_pos[2]
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        path_msg.poses.append(pose_msg)
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = self.target_pos[0]
        pose_msg.pose.position.y = self.target_pos[1]
        pose_msg.pose.position.z = self.target_pos[2]
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        path_msg.poses.append(pose_msg)
        self.trajectory_path_pub.publish(path_msg)

        uav_nav_msg.target_pos_x = self.target_pos[0]
        uav_nav_msg.target_pos_y = self.target_pos[1]
        self.uav_nav_pub.publish(uav_nav_msg)

        root_pose_msg.position.x = self.target_pos[0]
        root_pose_msg.position.y = self.target_pos[1]
        self.ddp_root_pose_pub.publish(root_pose_msg)


if __name__ == "__main__":
    rospy.init_node("linear_motion_flight_demo")
    node = LinearMotionFlightDemo()
    node.main()
