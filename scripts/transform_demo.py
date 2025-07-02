#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, Empty, UInt8
import time
import math


class TransformDemo:
    def __init__(self):
        self.is_transforming = False
        self.flight_state = None

        self.target_ee_pos_pub = rospy.Publisher("target_ee_final_pos", Vector3Stamped, queue_size=1)
        self.start_pub = rospy.Publisher("teleop_command/start", Empty, queue_size=1)
        self.takeoff_pub = rospy.Publisher("teleop_command/takeoff", Empty, queue_size=1)

        is_transforming_sub = rospy.Subscriber("is_transforming", Bool, self.isTransformingCallback)
        flight_state_sub = rospy.Subscriber("flight_state", UInt8, self.flightStateCallback)

        time.sleep(2.0)
        self.r = rospy.Rate(10)

    def isTransformingCallback(self, msg):
        self.is_transforming = msg.data

    def flightStateCallback(self, msg):
        self.flight_state = msg.data

    def main(self):
        last_moving_time = rospy.Time.now().to_sec()
        cnt = 0
        dt = 2.0
        split_n = 4
        center_x = 1.4
        center_y = 0.0
        center_z = 0.0
        radian = 0.6

        while not rospy.is_shutdown():
            if self.flight_state is None:
                continue
            elif self.flight_state == 0:
                self.start_pub.publish(Empty())
                continue
            elif self.flight_state == 1:
                self.takeoff_pub.publish(Empty())
                last_moving_time = rospy.Time.now().to_sec() + 10.0  # temporal solution to initialize
                continue

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
