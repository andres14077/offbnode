#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import copy
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
from offbnode.msg import PlaneStamped
# import matplotlib.pyplot as plt
# import statistics
import numpy as np

class procesado_plc:
    def __init__(self):
        self.rate=rospy.Rate(20)
        self.planos=[]

        self.cmd_sub=rospy.Subscriber("offbnode/procesado_on", Bool, self.cmd_cb)
        self.plane_sub=rospy.Subscriber('offbnode/plane_individual_in_map', PlaneStamped, self.plane_individual_in_map_cb)

        self.plano_in_local_pub=rospy.Publisher('offbnode/plano_in_local', PlaneStamped, queue_size=10)
        self.procesado_completed_pub=rospy.Publisher('offbnode/procesado_completed', Bool, queue_size=10 )
        self.plane_pub=rospy.Publisher('offbnode/plano_promedio_in_map', PlaneStamped, queue_size=10)

    def cmd_cb(self,msg):
        self.point_cloud_sub=rospy.Subscriber("offbnode/points2", PointCloud2, self.point_cloud_cb,queue_size=1)

    def Plano_Z(self,p,v,x,y):
        return p.z-(v.x*(x-p.x)+v.y*(y-p.y))/v.z

    def point_cloud_cb(self,msg):
        self.point_cloud_sub.unregister()
        plane=PlaneStamped()
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
        plane.header.frame_id="cgo3_camera_optical_link"
        plane.header.stamp = rospy.Time.now()
        plane.point.point.z = b[0]
        plane.vector.vector.x = b[1]*-1
        plane.vector.vector.y = b[2]*-1
        plane.vector.vector.z = 1
        self.plano_in_local_pub.publish(plane)

    def plane_individual_in_map_cb(self,msg):
        self.planos.append(copy.deepcopy(msg))
        plane=PlaneStamped()
        z=[]
        x=[]
        y=[]
        for i in self.planos:
            for j in range(-20,20):
                for k in range(-20,20):
                    x.append(j)
                    y.append(k)
                    z.append(self.Plano_Z(i.point.point,i.vector.vector,j,k))
        X = np.array([x,y])
        X = np.insert(X, 0, np.array((np.ones(len(X[0])))), 0).T
        Z = np.array(z)
        b = np.linalg.inv(X.T @ X) @ X.T @ Z
        plane.header.frame_id="map"
        plane.header.stamp = rospy.Time.now()
        plane.point.point.z = b[0]
        plane.vector.vector.x = b[1]*-1
        plane.vector.vector.y = b[2]*-1
        plane.vector.vector.z = 1
        self.plane_pub.publish(plane)
        procesado_completed=Bool(True)
        self.procesado_completed_pub.publish(procesado_completed)

if __name__ == '__main__':
    rospy.init_node('procesado_plc', anonymous=True)
    nodo=procesado_plc()
    rospy.spin()