#!/usr/bin/env python3
"""Save FAST-LIO odometry as a TUM trajectory in the mapping frame."""

import math
import os

import rospy
from nav_msgs.msg import Odometry


def main():
    rospy.init_node("fastlio_tum_recorder")
    output = os.path.abspath(os.path.expanduser(rospy.get_param("~output")))
    topic = rospy.get_param("~odom_topic", "/Odometry")
    os.makedirs(os.path.dirname(output), exist_ok=True)
    count = 0
    with open(output, "w", encoding="utf-8", buffering=1) as stream:
        stream.write("# timestamp tx ty tz qx qy qz qw\n")

        def record(msg):
            nonlocal count
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            values = (msg.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            if not all(math.isfinite(value) for value in values):
                rospy.logwarn_throttle(5.0, "FAST-LIO odometry contains non-finite values")
                return
            stream.write("{:.9f} {:.6f} {:.6f} {:.6f} {:.8f} {:.8f} {:.8f} {:.8f}\n".format(*values))
            count += 1

        rospy.Subscriber(topic, Odometry, record, queue_size=1000)
        rospy.loginfo("Recording %s as TUM: %s", topic, output)
        rospy.spin()
    rospy.loginfo("Saved %d TUM poses: %s", count, output)


if __name__ == "__main__":
    main()
