#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler

class procesado_plc_2:
    def __init__(self):
        self.rate=rospy.Rate(20)
        self.point_sub=rospy.Subscriber('offbnode/pose_in_plane', PoseStamped, self.point_cb )
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.t=[]
    def point_cb(self,msg):
        t=TransformStamped()
        t.header.frame_id = "map"
        t.child_frame_id = "plano_terreno_" + str(msg.header.seq)
        # Armando punto
        point=PointStamped()
        point.header.frame_id=msg.header.frame_id
        point.point.x=msg.pose.position.x
        point.point.y=msg.pose.position.y
        point.point.z=msg.pose.position.z
        transform = self.tf_buffer.lookup_transform("map","cgo3_camera_optical_link",rospy.Time(),rospy.Duration(1.0))
        point_t = tf2_geometry_msgs.do_transform_point(point, transform)
        t.transform.translation.x = point_t.point.x
        t.transform.translation.y = point_t.point.y
        t.transform.translation.z = point_t.point.z
        # Armando vector
        vector=Vector3Stamped()
        vector.header.frame_id=msg.header.frame_id
        vector.vector.x=msg.pose.orientation.x
        vector.vector.y=msg.pose.orientation.y
        vector.vector.z=msg.pose.orientation.z
        vector_t = tf2_geometry_msgs.do_transform_vector3(vector, transform)
        yaw = math.atan2( vector_t.vector.y , vector_t.vector.x )
        pitch = math.atan2( -vector_t.vector.z , math.sqrt( vector_t.vector.x**2 + vector_t.vector.y**2 ) )
        q=quaternion_from_euler(0,pitch-math.pi/2,yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.t.append(t)
    def update(self):
        for i in range(len(self.t)):
            self.t[i].header.stamp = rospy.Time.now()
            self.br.sendTransform(self.t[i])

if __name__ == '__main__':
    rospy.init_node('procesado_plc_2', anonymous=True)
    nodo=procesado_plc_2()
    while(not rospy.is_shutdown()):
        nodo.update()
        nodo.rate.sleep()
