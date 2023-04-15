#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from cv_bridge import CvBridge
import cv2


class Cap_imag:

    def __init__(self):
        rospy.loginfo("initializing tf_2")
        self._cv_bridge=CvBridge()
        rospy.wait_for_service("offbnode/tensor_flow_start")
        self.tensor_client = rospy.ServiceProxy("offbnode/tensor_flow_start", Empty)
        self.image_sub=rospy.Subscriber("iris_gimbal/resized/image", Image, self.Image_Calback)

    def Image_Calback(self,msg):
        cv_image=self._cv_bridge.imgmsg_to_cv2(msg, "rgb8")
        cv2.imwrite("/tmp/image_msg.png",cv_image)
        self.tensor_client()





if __name__ == '__main__':
    rospy.init_node('tensor_flow', anonymous=True)
    cap=Cap_imag()
    rospy.spin()

