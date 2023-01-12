#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt
import statistics
distancia_z=[]
t=[]
def point_cloud_cb(msg):
    global distancia_z
    global t
    z=[]
    for p in pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True):
        z.append(p[2])
    st_dev = statistics.pstdev(z)
    print (st_dev)
    dist=statistics.mean(z)
    rospy.loginfo("z_mean : %f",dist)
    distancia_z.append(dist)
    tnow=rospy.Time.now()
    t.append(tnow.secs)

if __name__ == '__main__':
    rospy.init_node('procesado_plc', anonymous=True)
    rate=rospy.Rate(20)
    local_pose_sub=rospy.Subscriber("offbnode/points2", PointCloud2, point_cloud_cb)


    while(not rospy.is_shutdown()):
        
        plt.plot(distancia_z)
        plt.pause(0.0001)
        rate.sleep()
