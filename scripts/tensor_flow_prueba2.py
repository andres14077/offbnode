#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from mavros_msgs.msg import WaypointReached
from mavros_msgs.msg import MountControl
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from cv_bridge import CvBridge
import cv2
import sys
import os


class Cap_imag:

    def __init__(self):
        rospy.loginfo("init node capture images for offboard")
        rospy.Subscriber("/iris_gimbal/usb_cam/image_raw", Image, self.Image_Calback)
        self.tensor_pub = rospy.Publisher("offbnode/tensor_flow_start", Empty,queue_size=10)
        self._cv_bridge=CvBridge()


    def Image_Calback(self,image):
        cv_image=self._cv_bridge.imgmsg_to_cv2(image, "bgr8")
        # cv2.imwrite("output.png", img_out)
        # cv_image=cv2.resize(cv_image,(227,227))
        # name=self.image_directory+"/Captura_No_"+str(self.last_id)+".png"
        cv2.imwrite("/tmp/image_msg.png",cv_image)
        self.tensor_pub.publish()
        rospy.loginfo("Foto guardada")
        # rospy.loginfo("captura de imagen "+str(self.last_id))





if __name__ == '__main__':
    rospy.init_node('tensor_flow', anonymous=True)
    cap=Cap_imag()
    rospy.spin()

