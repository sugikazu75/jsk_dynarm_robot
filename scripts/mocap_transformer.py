#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from tf.transformations import quaternion_matrix, quaternion_from_matrix
import numpy as np
import tf2_ros


def pose_to_matrix(pose):
    translation = [pose.position.x, pose.position.y, pose.position.z]
    rotation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    matrix = quaternion_matrix(rotation)
    matrix[0:3, 3] = translation
    return matrix


class MocapTransformer:
    def __init__(self):
        self.frame_name = rospy.get_param("~frame_name")
        self.root_mocap_sub = rospy.Subscriber("root/mocap/pose", PoseStamped, self.rootMocapCallback)
        self.frame_mocap_sub = rospy.Subscriber(self.frame_name + "/mocap/pose", PoseStamped, self.frameMocapCallback)
        self.root_to_frame_pub = rospy.Publisher("root_to_" + self.frame_name + "/pose", PoseStamped, queue_size=10)
        self.br = tf2_ros.TransformBroadcaster()
        self.node_ns = rospy.get_namespace().strip("/")

        self.world_to_root = pose_to_matrix(Pose())
        self.world_to_frame = pose_to_matrix(Pose())
        self.received_root = False
        self.received_frame = False

        self.root_to_frame = None

        self.r = rospy.Rate(100)

    def rootMocapCallback(self, msg):
        self.world_to_root = msg
        self.received_root = True

    def frameMocapCallback(self, msg):
        self.world_to_frame = msg
        self.received_frame = True

    def main(self):
        while not rospy.is_shutdown():
            if self.received_root and self.received_frame:
                root_matrix = pose_to_matrix(self.world_to_root.pose)
                frame_matrix = pose_to_matrix(self.world_to_frame.pose)

                root_to_frame_matrix = np.linalg.inv(root_matrix).dot(frame_matrix)

                translation = root_to_frame_matrix[0:3, 3]
                rotation = quaternion_from_matrix(root_to_frame_matrix)

                msg = PoseStamped()
                msg.header.stamp = rospy.Time.now()
                msg.pose.position.x = translation[0]
                msg.pose.position.y = translation[1]
                msg.pose.position.z = translation[2]
                msg.pose.orientation.x = rotation[0]
                msg.pose.orientation.y = rotation[1]
                msg.pose.orientation.z = rotation[2]
                msg.pose.orientation.w = rotation[3]
                self.root_to_frame_pub.publish(msg)

                tf_msg = TransformStamped()
                tf_msg.header.stamp = rospy.Time.now()
                tf_msg.header.frame_id = self.node_ns + "/root"
                tf_msg.child_frame_id = self.node_ns + "/" + self.frame_name
                tf_msg.transform.translation.x = translation[0]
                tf_msg.transform.translation.y = translation[1]
                tf_msg.transform.translation.z = translation[2]
                tf_msg.transform.rotation.x = rotation[0]
                tf_msg.transform.rotation.y = rotation[1]
                tf_msg.transform.rotation.z = rotation[2]
                tf_msg.transform.rotation.w = rotation[3]
                self.br.sendTransform(tf_msg)

            self.r.sleep()


if __name__ == "__main__":
    rospy.init_node("mocap_transformer_node")
    mocap_transformer = MocapTransformer()
    mocap_transformer.main()
