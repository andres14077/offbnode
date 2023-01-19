#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion

class procesado_plc_2:
    def __init__(self):
        rospy.init_node('procesado_plc_2', anonymous=True)
        self.rate=rospy.Rate(20)
        self.point_sub=rospy.Subscriber('offbnode/point_in_plane', PointStamped, queue_size=10)
        self.vector_sub=rospy.Subscriber('offbnode/vector_in_plane', Vector3Stamped, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    def point_cloud_cb(self,msg):
        vector_normal=Vector3Stamped()
        vectorN=Vector3()
        point_plane=PointStamped()
        point_plane.header.frame_id="cgo3_camera_optical_link"
        point_plane.header.stamp = rospy.Time.now()
        point_plane.point.z = b[0]

        vector_normal.header.frame_id="cgo3_camera_optical_link"
        vector_normal.header.stamp = rospy.Time.now()
        vector_normal.vector.x = b[1]*-1
        vector_normal.vector.y = b[2]*-1
        vector_normal.vector.z = 1


if __name__ == '__main__':
    
    nodo=procesado_plc_2()
    while(not rospy.is_shutdown()):
        nodo.rate.sleep()
