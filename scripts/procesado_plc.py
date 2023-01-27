#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
# import matplotlib.pyplot as plt
# import statistics
import numpy as np

class procesado_plc:
    def __init__(self):
        self.rate=rospy.Rate(20)
        self.cmd_sub=rospy.Subscriber("offbnode/procesado_on", Bool, self.cmd_cb)
        self.point_pub=rospy.Publisher('offbnode/pose_in_plane', PoseStamped, queue_size=10)
    def cmd_cb(self,msg):
        self.point_cloud_sub=rospy.Subscriber("offbnode/points2", PointCloud2, self.point_cloud_cb,queue_size=1)
    def point_cloud_cb(self,msg):
        self.point_cloud_sub.unregister()
        point_plane=PoseStamped()
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
        point_plane.pose.position.z = b[0]

        point_plane.pose.orientation.x = b[1]*-1
        point_plane.pose.orientation.y = b[2]*-1
        point_plane.pose.orientation.z = 1

        self.point_pub.publish(point_plane)

if __name__ == '__main__':
    rospy.init_node('procesado_plc', anonymous=True)
    nodo=procesado_plc()
    rospy.spin()