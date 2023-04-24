#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from visualization_msgs.msg import Marker

def publish_marker():
    marker = Marker()
    marker.header.frame_id = "/map"  # El marco de referencia en el que se ubicará el modelo
    marker.header.stamp = rospy.Time.now()
    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = -63.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.mesh_resource = "package://mavlink_sitl_gazebo/models/l1/untitled.dae"  # Ruta al archivo del modelo 3D
    marker.id = 0

    marker_publisher = rospy.Publisher("/big_box", Marker, queue_size=10)

    while not rospy.is_shutdown():
        marker_publisher.publish(marker)
        rospy.sleep(1)

if __name__ == "__main__":
    rospy.init_node("mesh_marker_publisher")
    publish_marker()
