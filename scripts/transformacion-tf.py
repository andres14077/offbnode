#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


rospy.init_node('tf2_transformation')
tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

def example_function():
    pose_stamped=PoseStamped()
    pose_stamped.header.frame_id="cgo3_camera_link"
    pose_stamped.pose.orientation.x=0.7071
    pose_stamped.pose.orientation.z=0.7071

    transform = tf_buffer.lookup_transform("map","cgo3_camera_link",rospy.Time(),rospy.Duration(1.0))

    pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
    orientation_q = pose_transformed.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (pose_transformed.pose.position.x, pose_transformed.pose.position.y, pose_transformed.pose.position.z) = euler_from_quaternion (orientation_list)
    pose_transformed.pose.position.x*=57.29577951
    pose_transformed.pose.position.y*=57.29577951
    pose_transformed.pose.position.z*=57.29577951
    # print (pose_transformed.pose.position.x)
    return (pose_transformed)

if __name__ == '__main__':
    rate=rospy.Rate(20)
    local_pos_pub = rospy.Publisher("offbnode/camera_pose", PoseStamped, queue_size=10)

    while(not rospy.is_shutdown()):
        local_pos_pub.publish(example_function())
        rate.sleep()