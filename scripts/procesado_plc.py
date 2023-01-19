#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import statistics
import numpy as np
distancia_z=[]
t=[]
class procesado_plc:
    def __init__(self):
        rospy.init_node('procesado_plc', anonymous=True)
        self.rate=rospy.Rate(20)
        self.local_pose_sub=rospy.Subscriber("offbnode/points2", PointCloud2, self.point_cloud_cb)
        self.distancia_z=[]
        self.t=[]
        self.point_pub=rospy.Publisher('offbnode/point_in_plane', PointStamped, queue_size=10)
        self.vector_pub=rospy.Publisher('offbnode/vector_in_plane', Vector3Stamped, queue_size=10)
    def point_cloud_cb(self,msg):
        vector_normal=Vector3Stamped()
        vectorN=Vector3()
        point_plane=PointStamped()
        z=[]
        x=[]
        y=[]
        for p in pc2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True):
            z.append(p[2])
            x.append(p[0])
            y.append(p[1])
        st_dev = statistics.pstdev(z)
        # if (st_dev > 30):
        #     return
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

        self.point_pub.publish(point_plane)
        self.vector_pub.publish(vector_normal)

        print (b)
        #st_dev = statistics.pstdev(z)
        # print (st_dev)
        dist=statistics.mean(z)
        rospy.loginfo("z_mean : %f",dist)
        # distancia_z.append(dist)
        # tnow=rospy.Time.now()
        # t.append(tnow.secs)

if __name__ == '__main__':
    
    nodo=procesado_plc()
    while(not rospy.is_shutdown()):
        
        # plt.plot(distancia_z)
        # plt.pause(0.0001)
        nodo.rate.sleep()
