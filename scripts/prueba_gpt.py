#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, PointField
import cv2
import numpy as np
import struct

class DepthToPointCloud:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/iris_gimbal/resized/image', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/iris_gimbal/resized/camera_info', CameraInfo, self.camera_info_callback)
        self.depth_sub = rospy.Subscriber('/offbnode/depth/image_raw', Image, self.depth_callback)
        self.point_cloud_pub = rospy.Publisher('/camera/depth/points', PointCloud2, queue_size=10)
        self.K = None
        self.rgb_image = None
        self.depth_image = None

    def camera_info_callback(self, camera_info):
        self.K = np.reshape(camera_info.K, (3, 3))

    def image_callback(self, rgb_image):
        if self.K is None or self.depth_image is None:
            return
        depth_array = self.bridge.imgmsg_to_cv2(self.depth_image, desired_encoding="passthrough")
        depth_array = cv2.convertScaleAbs(depth_array)
        points_3d = cv2.reprojectImageTo3D(depth_array, self.K)
        colors = self.bridge.imgmsg_to_cv2(rgb_image, desired_encoding="rgb8")
        points_with_color = np.zeros((points_3d.shape[0], points_3d.shape[1], 6), dtype=np.float32)
        points_with_color[..., :3] = points_3d
        points_with_color[..., 3:] = colors[...,:3]
        self.publish_point_cloud(points_with_color)

    def depth_callback(self, depth_image):
        self.depth_image = depth_image

    def publish_point_cloud(self, points_with_color):
        msg = PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_depth_optical_frame"
        msg.height = points_with_color.shape[0]
        msg.width = points_with_color.shape[1]
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="r", offset=12, datatype=PointField.UINT8, count=1),
            PointField(name="g", offset=13, datatype=PointField.UINT8, count=1),
            PointField(name="b", offset=14, datatype=PointField.UINT8, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 24
        msg.row_step = msg.point_step * points_with_color.shape[1]
        msg.is_dense = True
        msg.data = points_with_color.tostring()
        self.point_cloud_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('depth_to_point_cloud_node')
    depth_to_point_cloud = DepthToPointCloud()
    rospy.spin()
