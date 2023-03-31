#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from mavros_msgs.msg import WaypointReached
from mavros_msgs.msg import MountControl
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import tensorflow as tf
import tensorflow_hub as hub
import matplotlib.pyplot as plt
import sys
import os


class Cap_imag:

    def __init__(self):
        rospy.loginfo("init node capture images for offboard")
        rospy.Subscriber("/iris_gimbal/usb_cam/image_raw", Image, self.Image_Calback)
        self._cv_bridge=CvBridge()
        self.module = hub.load("https://tfhub.dev/intel/midas/v2/2", tags=['serve'])

        self.image_directory=sys.argv[1]

        if not (os.path.exists(self.image_directory)):
            rospy.logerr("Carpeta no encontrada:"+self.image_directory)
            exit()

    def Image_Calback(self,image):
        cv_image=self._cv_bridge.imgmsg_to_cv2(image, "bgr8")
        img_resized = tf.image.resize(cv_image, [384,384], method='bicubic', preserve_aspect_ratio=False)
        img_resized = tf.transpose(img_resized, [2, 0, 1])
        img_input = img_resized.numpy()
        reshape_img = img_input.reshape(1,3,384,384)
        tensor = tf.convert_to_tensor(reshape_img, dtype=tf.float32)

        output = self.module.signatures['serving_default'](tensor)
        prediction = output['default'].numpy()
        prediction = prediction.reshape(384, 384)

        prediction = cv2.resize(prediction, (cv_image.shape[1], cv_image.shape[0]), interpolation=cv2.INTER_CUBIC)
        print(" Write image to: output.png")
        depth_min = prediction.min()
        depth_max = prediction.max()
        img_out = (255 * (prediction - depth_min) / (depth_max - depth_min)).astype("uint8")

        print(depth_min)
        print(depth_max)
        # cv2.imwrite("output.png", img_out)
        plt.imshow(img_out)

        # cv_image=cv2.resize(cv_image,(227,227))
        # name=self.image_directory+"/Captura_No_"+str(self.last_id)+".png"
        # cv2.imwrite(name,cv_image)
        # rospy.loginfo("captura de imagen "+str(self.last_id))





if __name__ == '__main__':
    rospy.init_node('tensor_flow', anonymous=True)
    cap=Cap_imag()
    rospy.spin()

