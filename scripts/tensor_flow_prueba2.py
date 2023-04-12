#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from cv_bridge import CvBridge
import numpy as np
import cv2
import sys
import os


class Cap_imag:

    def __init__(self):
        rospy.loginfo("initializing tf_2")
        self._cv_bridge=CvBridge()
        rospy.wait_for_service("offbnode/tensor_flow_start")
        self.tensor_client = rospy.ServiceProxy("offbnode/tensor_flow_start", Empty)
        self.image_sub=rospy.Subscriber("iris_gimbal/usb_cam/image_raw", Image, self.Image_Calback)
        self.depth_image_pub = rospy.Publisher("offbnode/depth_image", Image,queue_size=10)


    def Image_Calback(self,msg):
        cv_image=self._cv_bridge.imgmsg_to_cv2(msg, "rgb8")
        # print(cv_image)
        cv2.imwrite("/tmp/image_msg.png",cv_image)
        rospy.loginfo("Foto guardada")
        self.tensor_client()
        img=cv2.imread("/tmp/output.png")
        # print(img)
        msg_image=self._cv_bridge.cv2_to_imgmsg(img)
        msg_image.header.frame_id=msg.header.frame_id
        self.depth_image_pub.publish(msg_image)





if __name__ == '__main__':
    rospy.init_node('tensor_flow', anonymous=True)
    cap=Cap_imag()
    rospy.spin()

