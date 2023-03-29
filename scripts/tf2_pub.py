#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

def tf_callback(msg):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = msg.header.stamp
    t.header.frame_id = msg.header.frame_id
    t.child_frame_id = msg.child_frame_id
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf2_pub')
    rospy.Subscriber("mavros/global_position/local", Odometry, tf_callback, queue_size=5)
    rospy.spin()