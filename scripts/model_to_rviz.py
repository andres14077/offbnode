#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from visualization_msgs.msg import Marker
import sys
import rospkg
from copy import deepcopy

def publish_marker():
    if (sys.argv[1] == "pruebas"):
        exit(0)
    rospack = rospkg.RosPack()
    archivo = rospack.get_path('offbnode')+"/models/" + sys.argv[1] + "/" + sys.argv[1] + ".sdf"
    marker_publisher = rospy.Publisher("/big_box", Marker, queue_size=10)
    marker2_publisher = rospy.Publisher("/big_box_2", Marker, queue_size=10)
    marker3_publisher = rospy.Publisher("/big_box_3", Marker, queue_size=10)

    marker = Marker()
    marker.header.frame_id = "/map"  # El marco de referencia en el que se ubicará el modelo
    marker.header.stamp = rospy.Time.now()
    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.8
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.mesh_resource = "package://offbnode/models/" + sys.argv[1] + "/untitled.dae"  # Ruta al archivo del modelo 3D
    marker.id = 0

    try:
        with open(archivo, 'r') as f:
            lines = f.readlines()
        for line in lines:
            if "pose" in line:
                parts = line.replace("<pose>","").split()
                marker.pose.position.x = float(parts[0])
                marker.pose.position.y = float(parts[1])
                marker.pose.position.z = float(parts[2])
                break
    except:
        pass

    marker2 = deepcopy(marker)
    marker2.color.r = 0.0
    marker2.color.g = 0.0
    marker2.color.b = 0.0
    marker2.mesh_resource = "package://offbnode/models/" + sys.argv[1] + "/plane.dae"  # Ruta al archivo del modelo 3D
    marker2.pose.position.z += 5

    marker3 = deepcopy(marker)
    marker3.color.r = 0.0
    marker3.color.g = 0.0
    marker3.color.b = 0.0
    marker3.mesh_resource = "package://offbnode/models/" + sys.argv[1] + "/plane2.dae"  # Ruta al archivo del modelo 3D


    while not rospy.is_shutdown():
        marker_publisher.publish(marker)
        marker2_publisher.publish(marker2)
        marker3_publisher.publish(marker3)
        rospy.sleep(1)

if __name__ == "__main__":
    rospy.init_node("mesh_marker_publisher")
    publish_marker()
