#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32
# import matplotlib.pyplot as plt
# import statistics
import numpy as np

class procesado_plc:
    def __init__(self):
        rospy.init_node('procesado_plc', anonymous=True)
        self.rate=rospy.Rate(20)
        self.numero=Int32()
        self.point_cloud_sub=rospy.Subscriber("offbnode/points2", PointCloud2, self.point_cloud_cb)
        self.point_pub=rospy.Publisher('offbnode/point_in_plane', PointStamped, queue_size=10)
        self.int_pub=rospy.Publisher('offbnode/num_of_plane', Int32, queue_size=10)
        self.vector_pub=rospy.Publisher('offbnode/vector_in_plane', Vector3Stamped, queue_size=10)
    def point_cloud_cb(self,msg):
        vector_normal=Vector3Stamped()
        point_plane=PointStamped()
        self.numero.data+=1
        z=[]
        x=[]
        y=[]
        for p in pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True):
            z.append(p[2])
            x.append(p[0])
            y.append(p[1])
        X = np.array([x,y])
        X = np.insert(X, 0, np.array((np.ones(len(X[0])))), 0).T
        Z = np.array(z)
        b = np.linalg.inv(X.T @ X) @ X.T @ Z
        point_plane.header.frame_id="cgo3_camera_optical_link"
        point_plane.header.stamp = rospy.Time.now()
        point_plane.point.z = b[0]

        vector_normal.header.frame_id="cgo3_camera_optical_link"
        vector_normal.header.stamp = rospy.Time.now()
        vector_normal.vector.x = b[1]*-1
        vector_normal.vector.y = b[2]*-1
        vector_normal.vector.z = 1
        self.int_pub.publish(self.numero)
        self.point_pub.publish(point_plane)
        self.vector_pub.publish(vector_normal)

if __name__ == '__main__':

    nodo=procesado_plc()
    rospy.spin()