#!/usr/bin/env python
# -*- coding: utf-8 -*-
from copy import copy
import math
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image,CameraInfo
from mavros_msgs.msg import MountControl
from mavros_msgs.srv import CommandBool,CommandBoolRequest,CommandBoolResponse
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge
import cv2
class Image_Joke:
    def __init__(self):
        #variables generales
        self.rate = rospy.Rate(20)
        self.toma_imagen_l=False
        self.toma_imagen_r=False
        self.image_lg = Image()
        self.image_rg = Image()
        self.image_l = Image()
        self.image_r = Image()
        self.current_local_pose=PoseStamped()
        self.set_pose=PoseStamped()
        self.set_pose.pose.position.z=50
        self._cv_bridge=CvBridge()
        self.control_stade=0
        # Informacion para camara 1 calibrada
        self.width = 921
        self.height = 691
        self.camera_info_l = CameraInfo()
        self.camera_info_l.header.frame_id="cgo3_camera_optical_link"
        self.camera_info_l.width = self.width
        self.camera_info_l.height = self.height
        self.camera_info_l.distortion_model="plumb_bob"
        self.camera_info_l.D=[-0.001093943834000625, -0.003399530263584313, 3.355356194349521e-05, 0.00045497470336154714, 0.0]
        self.camera_info_l.K=[738.0459022620655, 0.0, 464.01502507967683,
                            0.0, 738.0375895328136, 349.0653242045357,
                            0.0, 0.0, 1.0]
        self.camera_info_l.R=[0.9997015918831174, -0.009447704302609411, 0.022527051954742573,
                            0.00942988069258523, 0.999955135088262, 0.0008973074943409966,
                            -0.022534518776419956, -0.0006846123182124598, 0.9997458310838255]
        self.camera_info_l.P=[756.4338810928781, 0.0, 440.56248474121094, 0.0,
                            0.0, 756.4338810928781, 348.1920471191406, 0.0,
                            0.0, 0.0, 1.0, 0.0]
        # Informacion para camara 2 calibrada
        self.camera_info_r = CameraInfo()
        self.camera_info_r.header.frame_id="cgo3_camera_optical_link"
        self.camera_info_r.width = self.width
        self.camera_info_r.height = self.height
        self.camera_info_r.distortion_model="plumb_bob"
        self.camera_info_r.D=[-0.0006410119678569111, 0.003426125113731216, -0.00012307077116445515, 0.0001696197294923834, 0.0]
        self.camera_info_r.K=[735.754824877442, 0.0, 459.63707703729636,
                            0.0, 735.8441084976562, 347.59097532621905,
                            0.0, 0.0, 1.0]
        self.camera_info_r.R=[0.9997590702781092, -0.0098528580313329, 0.01961434641442733,
                            0.009868371202169487, 0.9999510654193599, -0.0006942740836475667,
                            -0.01960654601063, 0.000887668463691183, 0.9998073791477194]
        self.camera_info_r.P=[756.4338810928781, 0.0, 440.56248474121094, -1450.304877800299,
                            0.0, 756.4338810928781, 348.1920471191406, 0.0,
                            0.0, 0.0, 1.0, 0.0]

        # Posision objetivo
        self.pose_local=PoseStamped()
        q=quaternion_from_euler(0,0,0)
        self.pose_local.pose.orientation.x = q[0]
        self.pose_local.pose.orientation.y = q[1]
        self.pose_local.pose.orientation.z = q[2]
        self.pose_local.pose.orientation.w = q[3]
        # orientacion camara objetivo
        self.camera_pose=MountControl()
        self.camera_pose.header.frame_id=""
        self.camera_pose.mode=2
        self.camera_pose.pitch=-90
        self.camera_pose.roll=0
        self.camera_pose.yaw=0

        #suscribtores y publicadores
        self.image_raw_sub=rospy.Subscriber("iris_gimbal/usb_cam/image_raw", Image, self.image_cb)
        self.local_pose_sub=rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_pose_cb)
        self.set_pose_sub=rospy.Subscriber("offbnode/set_pose_cmd", PoseStamped, self.set_pose_cb)

        self.local_poss_pub = rospy.Publisher("offbnode/pose_local_cmd", PoseStamped, queue_size=10)
        self.camera_pose_pub=rospy.Publisher('mavros/mount_control/command', MountControl, queue_size=10)

        self.image_left_pub = rospy.Publisher("offbnode/left/image_raw", Image, queue_size=10)
        self.image_right_pub = rospy.Publisher("offbnode/right/image_raw", Image, queue_size=10)

        self.camera_info_left_pub = rospy.Publisher("offbnode/left/camera_info", CameraInfo, queue_size=10)
        self.camera_info_right_pub = rospy.Publisher("offbnode/right/camera_info", CameraInfo, queue_size=10)

        # sincronizacion con servicio
        rospy.loginfo("Esperando servicio offboard")
        rospy.wait_for_service("offbnode/master_ok")
        offboard_client = rospy.ServiceProxy("offbnode/master_ok", CommandBool)
        offboard_cmd = CommandBoolRequest()
        offboard_cmd.value = True
        while(offboard_client.call(offboard_cmd).success == False):
            self.rate.sleep()
    def image_cb(self,msg):
        if (self.toma_imagen_l==True):
            self.toma_imagen_l = False
            self.image_lg = copy(msg)
        if (self.toma_imagen_r==True):
            self.toma_imagen_r = False
            self.image_rg = copy(msg)
    def local_pose_cb(self,msg):
        self.current_local_pose = msg
    def set_pose_cb(self,msg):
        self.set_pose = msg
    def distancia_to_setpoint(self,pose):
        d= (self.current_local_pose.pose.position.x-pose.pose.position.x)*(self.current_local_pose.pose.position.x-pose.pose.position.x)
        d+=(self.current_local_pose.pose.position.y-pose.pose.position.y)*(self.current_local_pose.pose.position.y-pose.pose.position.y)
        d+=(self.current_local_pose.pose.position.z-pose.pose.position.z)*(self.current_local_pose.pose.position.z-pose.pose.position.z)*16
        return math.sqrt(d)
    def update(self):
        #publicar mensajes de estado
        self.local_poss_pub.publish(self.pose_local)
        self.camera_pose_pub.publish(self.camera_pose)
        self.image_l.header.stamp = rospy.Time.now()
        self.camera_info_l.header.stamp = rospy.Time.now()
        self.image_r.header.stamp = rospy.Time.now()
        self.camera_info_r.header.stamp = rospy.Time.now()
        self.image_left_pub.publish(self.image_l)
        self.camera_info_left_pub.publish(self.camera_info_l)
        self.image_right_pub.publish(self.image_r)
        self.camera_info_right_pub.publish(self.camera_info_r)

        # Actualizar estado del controlador
        if(self.control_stade==0):
            self.control_stade+=1
            rospy.loginfo("Ir a posicion 1")
            self.pose_local.pose.position.x = 0 + self.set_pose.pose.position.x
            self.pose_local.pose.position.y = 2 + self.set_pose.pose.position.y
            self.pose_local.pose.position.z = 0 + self.set_pose.pose.position.z
        elif(self.control_stade==1):
            if(self.distancia_to_setpoint(self.pose_local)<0.05):
                self.control_stade+=1
                self.toma_imagen_l= True
        elif(self.control_stade==2):
            if(not self.toma_imagen_l):
                self.control_stade+=1
        elif(self.control_stade==3):
            self.control_stade+=1
            rospy.loginfo("Ir a posicion 2")
            self.pose_local.pose.position.x = 0 + self.set_pose.pose.position.x
            self.pose_local.pose.position.y = 0 + self.set_pose.pose.position.y
            self.pose_local.pose.position.z = 0 + self.set_pose.pose.position.z
        elif(self.control_stade==4):
            if(self.distancia_to_setpoint(self.pose_local)<0.05):
                self.control_stade+=1
                self.toma_imagen_r= True
        elif(self.control_stade==5):
            if(not self.toma_imagen_r):
                self.control_stade+=1
        elif(self.control_stade==6):
            self.control_stade+=1
        elif(self.control_stade==7):
            self.control_stade=0
            image_rl=self._cv_bridge.imgmsg_to_cv2(self.image_rg, "bgr8")
            image_rl=cv2.resize(image_rl,(self.width, self.height))
            image_r2=self._cv_bridge.cv2_to_imgmsg(image_rl, "bgr8")
            image_ll=self._cv_bridge.imgmsg_to_cv2(self.image_lg, "bgr8")
            image_ll=cv2.resize(image_ll,(self.width, self.height))
            image_l2=self._cv_bridge.cv2_to_imgmsg(image_ll, "bgr8")
            self.image_l=copy(image_l2)
            self.image_r=copy(image_r2)

if __name__ == "__main__":
    rospy.init_node("image_joke_py")

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)
    nodo=Image_Joke()
    while(not rospy.is_shutdown()):
        nodo.update()
        nodo.rate.sleep()





