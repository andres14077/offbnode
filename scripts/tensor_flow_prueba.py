#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_srvs.srv import Empty,EmptyResponse
import cv2
import tensorflow as tf
import tensorflow_hub as hub
import matplotlib.pyplot as plt
import numpy as np


class Cap_imag:

    def __init__(self):
        rospy.loginfo("initializing tf")
        # self.module = hub.load("https://tfhub.dev/intel/midas/v2/2", tags=['serve'])
        self.interpreter = tf.lite.Interpreter(model_path="/root/catkin_ws/src/offbnode/scripts/model_opt.tflite")
        self.interpreter.allocate_tensors()
        self.input_details  = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.input_shape = self.input_details[0]['shape']
        self.tensor_service = rospy.Service("offbnode/tensor_flow_start", Empty, self.Tensor_Callback)


    def Tensor_Callback(self,msg):
        rospy.loginfo("leer foto")
        img=cv2.imread("/tmp/image_msg.png")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) / 255.0
        # print(img)

        img_resized = tf.image.resize(img, [256,256], method='bicubic', preserve_aspect_ratio=False)
        img_input = img_resized.numpy()
        mean=[0.485, 0.456, 0.406]
        std=[0.229, 0.224, 0.225]
        img_input = (img_input - mean) / std
        reshape_img = img_input.reshape(1,256,256,3)
        tensor = tf.convert_to_tensor(reshape_img, dtype=tf.float32)

        self.interpreter.set_tensor(self.input_details[0]['index'], tensor)
        self.interpreter.invoke()
        output = self.interpreter.get_tensor(self.output_details[0]['index'])
        output = output.reshape(256, 256)


        prediction = cv2.resize(output, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_CUBIC)
        depth_min = prediction.min()
        depth_max = prediction.max()
        # img_out = (255 * (prediction - depth_min) / (depth_max - depth_min)).astype("uint8")
        img_out = (65535 * (prediction - depth_min) / (depth_max - depth_min)).astype("uint16")
        print(depth_min)
        print(depth_max)
        # print(prediction)
        cv2.imwrite("/tmp/output.png", img_out)
        # print (prediction)
        # np.savetxt("/tmp/output.txt",prediction)
        # plt.imshow(img_out)
        # plt.show()
        return EmptyResponse()





if __name__ == '__main__':
    rospy.init_node('tensor_flow', anonymous=True)
    cap=Cap_imag()
    rospy.spin()

