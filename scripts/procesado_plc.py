#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import PointStamped
import matplotlib.pyplot as plt
import statistics
import numpy as np
distancia_z=[]
t=[]
def point_cloud_cb(msg):
    global distancia_z
    global t
    vector_normal=Vector3Stamped()
    point_plane=PointStamped()
    z=[]
    x=[]
    y=[]
    for p in pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True):
        z.append(p[2])
        x.append(p[0])
        y.append(p[1])
    st_dev = statistics.pstdev(z)
    if (st_dev > 30):
        return
    X = np.array([x,y])
    X = np.insert(X, 0, np.array((np.ones(len(X[0])))), 0).T
    Z = np.array(z)
    b = np.linalg.inv(X.T @ X) @ X.T @ Z
    print (b)
    #st_dev = statistics.pstdev(z)
    # print (st_dev)
    # dist=statistics.mean(z)
    # rospy.loginfo("z_mean : %f",dist)
    # distancia_z.append(dist)
    # tnow=rospy.Time.now()
    # t.append(tnow.secs)

if __name__ == '__main__':
    rospy.init_node('procesado_plc', anonymous=True)
    rate=rospy.Rate(20)
    local_pose_sub=rospy.Subscriber("offbnode/points2", PointCloud2, point_cloud_cb)


    while(not rospy.is_shutdown()):
        
        # plt.plot(distancia_z)
        # plt.pause(0.0001)
        rate.sleep()
