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
        rospy.init_node('procesado_plc_2', anonymous=True)
        self.rate=rospy.Rate(20)
        self.point_sub=rospy.Subscriber('offbnode/point_in_plane', PointStamped, self.point_cb )
        self.vector_sub=rospy.Subscriber('offbnode/vector_in_plane', Vector3Stamped, self.vector_cb)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()

        self.t = TransformStamped()
        self.t.transform.rotation.w = 1
        self.t.header.frame_id = "cgo3_camera_optical_link"
        self.t.child_frame_id = "plano_terreno"
    def point_cb(self,msg):
        self.t.transform.translation.x = msg.point.x
        self.t.transform.translation.y = msg.point.y
        self.t.transform.translation.z = msg.point.z
    def vector_cb(self,msg):
        # v_len=math.sqrt(msg.vector.x*msg.vector.x + msg.vector.y*msg.vector.y + msg.vector.z*msg.vector.z)
        yaw = math.atan2( msg.vector.y , msg.vector.x )
        pitch = math.atan2( -msg.vector.z , math.sqrt( msg.vector.x**2 + msg.vector.y**2 ) )
        q=quaternion_from_euler(0,pitch,yaw)
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]
    def update(self):
        self.t.header.stamp = rospy.Time.now()
        self.br.sendTransform(self.t)

if __name__ == '__main__':
    
    nodo=procesado_plc_2()
    while(not rospy.is_shutdown()):
        nodo.update()
        nodo.rate.sleep()
