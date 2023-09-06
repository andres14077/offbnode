#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from mavros_msgs.msg import WaypointReached
from mavros_msgs.msg import MountControl
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
#import tensorflow as tf
import sys
import os


class Cap_imag:

    def __init__(self):
        rospy.loginfo("init node capture images for offboard")
        rospy.Subscriber("/offbnode/mission/reached",WaypointReached,self.Waypoint_Reached_Calback)
        rospy.Subscriber("/iris_gimbal/usb_cam/image_raw", Image, self.Image_Calback)
        self._cv_bridge=CvBridge()
        self.last_id=0
        self.capture_image=0

        self.image_directory=sys.argv[1]

        if not (os.path.exists(self.image_directory)):
            rospy.logerr("Carpeta no encontrada:"+self.image_directory)
            exit()

    def Image_Calback(self,image):
        if self.capture_image:
            self.capture_image=0
            cv_image=self._cv_bridge.imgmsg_to_cv2(image, "bgr8")
            cv_image=cv2.resize(cv_image,(227,227))
            name=self.image_directory+"/Captura_No_"+str(self.last_id)+".png"
            cv2.imwrite(name,cv_image)
            rospy.loginfo("captura de imagen "+str(self.last_id))

    def Waypoint_Reached_Calback(self,waypoint):
        if((self.last_id!=waypoint.wp_seq) and (self.capture_image==0)):
            self.last_id=waypoint.wp_seq
            self.capture_image=1


if __name__ == '__main__':
    rospy.init_node('capture_for_px4', anonymous=True)
    cap=Cap_imag()
    rate=rospy.Rate(1)
    rospy.spin()



