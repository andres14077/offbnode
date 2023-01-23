#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

class procesado_plc_2:
    def __init__(self):
        self.rate=rospy.Rate(20)
        self.point_sub=rospy.Subscriber('offbnode/point_in_plane', PointStamped, self.point_cb )
        self.vector_sub=rospy.Subscriber('offbnode/vector_in_plane', Vector3Stamped, self.vector_cb)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()

        self.t = TransformStamped()
        self.t.transform.rotation.w = 1
        self.t.header.frame_id = "map"
        self.t.child_frame_id = "plano_terreno"
    def point_cb(self,msg):
        transform = self.tf_buffer.lookup_transform("map","cgo3_camera_optical_link",rospy.Time(),rospy.Duration(1.0))
        point_t = tf2_geometry_msgs.do_transform_point(msg, transform)
        self.t.transform.translation.x = point_t.point.x
        self.t.transform.translation.y = point_t.point.y
        self.t.transform.translation.z = point_t.point.z
    def vector_cb(self,msg):
        transform = self.tf_buffer.lookup_transform("map","cgo3_camera_optical_link",rospy.Time(),rospy.Duration(1.0))
        vector_t = tf2_geometry_msgs.do_transform_vector3(msg, transform)
        yaw = math.atan2( vector_t.vector.y , vector_t.vector.x )
        pitch = math.atan2( -vector_t.vector.z , math.sqrt( vector_t.vector.x**2 + vector_t.vector.y**2 ) )
        q=quaternion_from_euler(0,pitch-math.pi/2,yaw)
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]
    def update(self):
        self.t.header.stamp = rospy.Time.now()
        self.br.sendTransform(self.t)

if __name__ == '__main__':
    rospy.init_node('procesado_plc_2', anonymous=True)
    nodo=procesado_plc_2()
    while(not rospy.is_shutdown()):
        nodo.update()
        nodo.rate.sleep()
