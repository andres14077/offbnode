#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_srvs.srv import Empty,EmptyResponse
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
import rospkg
import cv2
import tensorflow as tf

class Cap_imag:

    def __init__(self):
        rospy.loginfo("initializing tf")
        self._cv_bridge=CvBridge()
        self.camera_info=CameraInfo()
        rospack = rospkg.RosPack()
        # self.module = hub.load("https://tfhub.dev/intel/midas/v2/2", tags=['serve'])
        self.interpreter = tf.lite.Interpreter(model_path=rospack.get_path('offbnode')+"/scripts/model_opt.tflite")
        self.interpreter.allocate_tensors()
        self.input_details  = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.input_shape = self.input_details[0]['shape']
        self.tensor_service = rospy.Service("offbnode/tensor_flow_start", Empty, self.Tensor_Callback)
        self.image_raw_sub=rospy.Subscriber("iris_gimbal/resized/camera_info", CameraInfo, self.camera_info_cb)
        self.depth_image_pub = rospy.Publisher("offbnode/depth/image_raw", Image,queue_size=10)
        self.depth_camera_info_pub = rospy.Publisher("offbnode/depth/camera_info", CameraInfo,queue_size=10)

    def camera_info_cb(self,msg):
        self.camera_info=msg

    def Tensor_Callback(self,msg):
        # rospy.loginfo("leer foto")
        img=cv2.imread("/tmp/image_msg.png")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) / 255.0

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


        img_inversa = cv2.resize(output, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_CUBIC)

        # depth_min = img_inversa.min()
        depth_max = img_inversa.max()
        img_profundidad = depth_max - img_inversa
        # img_out = (65535 * (prediction - depth_min) / (depth_max - depth_min)).astype("uint16")
        msg_image=self._cv_bridge.cv2_to_imgmsg(img_profundidad)
        msg_image.header.frame_id="iris_gimbal/cgo3_camera_optical_link"
        msg_image.header.stamp = rospy.Time.now()
        self.camera_info.header.stamp = rospy.Time.now()
        self.depth_image_pub.publish(msg_image)
        self.depth_camera_info_pub.publish(self.camera_info)
        return EmptyResponse()




if __name__ == '__main__':
    rospy.init_node('tensor_flow', anonymous=True)
    cap=Cap_imag()
    rospy.spin()

