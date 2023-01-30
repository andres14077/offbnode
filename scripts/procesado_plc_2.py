#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
from offbnode.msg import PlaneStamped
from std_msgs.msg import Bool
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler

class procesado_plc_2:
    def __init__(self):
        self.rate=rospy.Rate(20)
        self.pose_sub=rospy.Subscriber('offbnode/pose_in_plane', PlaneStamped, self.point_cb )
        self.plane_promedio_sub=rospy.Subscriber('offbnode/plane_in_map', PlaneStamped, self.plane_promedio_cb )
        self.procesado_completed_pub=rospy.Publisher('offbnode/procesado_completed', Bool, queue_size=10 )
        self.plane_pub=rospy.Publisher('offbnode/plane_local_in_map', PlaneStamped, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()
        self.t=[]
        t=TransformStamped()
        t.header.frame_id = "map"
        t.child_frame_id = "plano_promedio"
        t.transform.rotation.w = 1
        self.t.append(t)

    def point_cb(self,msg):
        plane=PlaneStamped()
        plane.header.frame_id = "map"
        t=TransformStamped()
        t.header.frame_id = "map"
        t.child_frame_id = "plano_terreno_" + str(msg.header.seq)
        # Armando punto
        point=PointStamped()
        point.header.frame_id=msg.header.frame_id
        point.point.x=msg.point.point.x
        point.point.y=msg.point.point.y
        point.point.z=msg.point.point.z
        transform = self.tf_buffer.lookup_transform("map","cgo3_camera_optical_link",rospy.Time(),rospy.Duration(1.0))
        point_t = tf2_geometry_msgs.do_transform_point(point, transform)
        t.transform.translation.x = point_t.point.x
        t.transform.translation.y = point_t.point.y
        t.transform.translation.z = point_t.point.z
        plane.point=point_t
        # Armando vector
        vector=Vector3Stamped()
        vector.header.frame_id=msg.header.frame_id
        vector.vector.x=msg.vector.vector.x
        vector.vector.y=msg.vector.vector.y
        vector.vector.z=msg.vector.vector.z
        vector_t = tf2_geometry_msgs.do_transform_vector3(vector, transform)
        yaw = math.atan2( vector_t.vector.y , vector_t.vector.x )
        pitch = math.atan2( -vector_t.vector.z , math.sqrt( vector_t.vector.x**2 + vector_t.vector.y**2 ) )
        q=quaternion_from_euler(0,pitch-math.pi/2,yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.t.append(t)
        plane.vector=vector_t
        self.plane_pub.publish(plane)

    def plane_promedio_cb(self,msg):
        t=TransformStamped()
        t.header.frame_id = self.t[0].header.frame_id
        t.child_frame_id = self.t[0].child_frame_id
        t.transform.translation.x = msg.point.point.x
        t.transform.translation.y = msg.point.point.y
        t.transform.translation.z = msg.point.point.z
        yaw = math.atan2( msg.vector.vector.y , msg.vector.vector.x )
        pitch = math.atan2( -msg.vector.vector.z , math.sqrt( msg.vector.vector.x**2 + msg.vector.vector.y**2 ) )
        q=quaternion_from_euler(0,pitch-math.pi/2,yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.t[0]=t

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
