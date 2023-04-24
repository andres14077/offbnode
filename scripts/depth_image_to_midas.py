#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
import std_msgs.msg as std_msgs
from cv_bridge import CvBridge
import cv2


class Depth_image_to_midas:

    def __init__(self):
        rospy.loginfo("initializing Depth_image_to_midas")
        self._cv_bridge=CvBridge()
        rospy.wait_for_service("offbnode/tensor_flow_start")
        self.tensor_client = rospy.ServiceProxy("offbnode/tensor_flow_start", Empty)
        self.depth_image_sub = rospy.Subscriber("offbnode/tomar_depth_image", std_msgs.Empty,self.depth_image_cb)

    def Image_Calback(self,msg):
        self.image_sub.unregister()
        cv_image=self._cv_bridge.imgmsg_to_cv2(msg, "rgb8")
        cv2.imwrite("/tmp/image_msg.png",cv_image)
        self.tensor_client()

    def depth_image_cb(self,msg):
        self.image_sub=rospy.Subscriber("iris_gimbal/resized/image", Image, self.Image_Calback)





if __name__ == '__main__':
    rospy.init_node('tensor_flow', anonymous=True)
    cap=Depth_image_to_midas()
    rospy.spin()

