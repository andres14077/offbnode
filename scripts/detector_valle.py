#!/usr/bin/env python
# -*- coding: utf-8 -*-
from copy import copy
import math
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image,CameraInfo
from mavros_msgs.msg import MountControl
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_srvs.srv import Trigger,TriggerRequest,TriggerResponse
from mavros_msgs.srv import CommandBool,CommandBoolRequest,CommandBoolResponse
from tf.transformations import quaternion_from_euler,euler_from_quaternion
import cv2
import numpy as np
class Detector_valle:
    def __init__(self):
        #variables generales
        self.rate = rospy.Rate(20)
        self.rate2 = rospy.Rate(1)
        self.current_local_pose=PoseStamped()
        self.set_pose=PoseStamped()
        self.set_pose.pose.position.z=10

        self.control_stade=0
        self.control_stade_A=0
        # Posision objetivo
        self.pose_local=PoseStamped()
        # orientacion camara objetivo
        self.camera_pose=MountControl()
        self.camera_pose.header.frame_id=""
        self.camera_pose.mode=2
        self.camera_pose.pitch=0
        self.camera_pose.roll=0
        self.camera_pose.yaw=0

        #suscribtores y publicadores
        self.local_pose_sub=rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_pose_cb)
        self.set_pose_sub=rospy.Subscriber("offbnode/set_point_to_measure_cmd", PoseStamped, self.set_pose_cb)
        self.procesado_completed_sub=rospy.Subscriber('offbnode/depth_ok', Empty, self.procesado_completed_cb )
        self.enviando_ruta_sub=rospy.Subscriber("/offbnode/enviando_tipo_terreno", Empty,self.enviando_ruta_cb)

        self.local_poss_pub = rospy.Publisher("offbnode/pose_local_cmd", PoseStamped, queue_size=10)
        self.camera_pose_pub=rospy.Publisher('mavros/mount_control/command', MountControl, queue_size=10)
        self.depth_image_pub = rospy.Publisher("offbnode/tomar_depth_image", Empty,queue_size=10)
        self.point_cloud_depth_start_pub=rospy.Publisher("/offbnode/depth_start", Empty,queue_size=10)
        self.point_cloud_depth_finish_pub=rospy.Publisher("/offbnode/depth_finish", Empty,queue_size=10)

        # iniciando servicio de medida de plano
        self.tomar_medida_service = rospy.Service("offbnode/identificar_terreno", Trigger,self.tomar_medida_cb)

        self.tomar_medida=False
        rospy.loginfo("Detector_valle init")
    def procesado_completed_cb(self,msg):
        self.procesado_in_progres=False
    def enviando_ruta_cb(self,msg):
        self.enviando_ruta=False
    def local_pose_cb(self,msg):
        self.current_local_pose = msg
    def set_pose_cb(self,msg):
        self.set_pose = msg
    def tomar_medida_cb(self,req):
        self.tomar_medida=True
        self.yaw_camera = 0
        self.control_stade=0
        self.point_cloud_depth_start_pub.publish()
        while(self.tomar_medida):
            self.update()
            self.rate.sleep()
        response=TriggerResponse()
        response.success=True
        return response
    def distancia_to_setpoint(self,pose):
        d= (self.current_local_pose.pose.position.x-pose.pose.position.x)*(self.current_local_pose.pose.position.x-pose.pose.position.x)
        d+=(self.current_local_pose.pose.position.y-pose.pose.position.y)*(self.current_local_pose.pose.position.y-pose.pose.position.y)
        d+=(self.current_local_pose.pose.position.z-pose.pose.position.z)*(self.current_local_pose.pose.position.z-pose.pose.position.z)
        return math.sqrt(d)
    def angulo_to_setpoint(self,pose):
        q = pose.pose.orientation
        orientation_list = [q.x, q.y, q.z, q.w]
        angulos1= euler_from_quaternion (orientation_list)[2]
        q = self.current_local_pose.pose.orientation
        orientation_list = [q.x, q.y, q.z, q.w]
        angulos2= euler_from_quaternion (orientation_list)[2]
        return abs(((angulos1 - angulos2 + np.pi) % (2*np.pi)) - np.pi)*180/np.pi
    def update(self):
        #publicar mensajes de estado
        self.local_poss_pub.publish(self.pose_local)
        self.camera_pose_pub.publish(self.camera_pose)
        if (self.control_stade_A != self.control_stade):
            self.control_stade_A = self.control_stade
            rospy.logdebug("Estado controlador detector de valle: "+ str(self.control_stade))
        # Actualizar estado del controlador
        if(self.control_stade==0):
            self.control_stade+=1
            self.pose_local.pose.position.x = self.set_pose.pose.position.x
            self.pose_local.pose.position.y = self.set_pose.pose.position.y
            self.pose_local.pose.position.z = self.set_pose.pose.position.z
            q=quaternion_from_euler(0,0,self.yaw_camera)
            self.pose_local.pose.orientation.x = q[0]
            self.pose_local.pose.orientation.y = q[1]
            self.pose_local.pose.orientation.z = q[2]
            self.pose_local.pose.orientation.w = q[3]
            self.camera_pose.yaw= (-1)*self.yaw_camera*180/np.pi
        elif(self.control_stade==1):
            if(self.distancia_to_setpoint(self.pose_local)<0.5):
                if(self.angulo_to_setpoint(self.pose_local)<1):
                    self.control_stade+=1
        elif(self.control_stade==2):
            self.yaw_camera+=15*np.pi/180
            self.procesado_in_progres=True
            self.depth_image_pub.publish()
            self.control_stade+=1
            self.intentos=0
        elif(self.control_stade==3):
            if(not self.procesado_in_progres):
                if(self.yaw_camera>(2*np.pi)):
                    self.point_cloud_depth_finish_pub.publish()
                    self.control_stade=4
                    self.enviando_ruta=True
                else:
                    self.control_stade=0
            self.intentos+=1
            # 10 segundos de espera
            if(self.intentos>200):
                self.control_stade=0
                self.tomar_medida=False

        elif(self.control_stade==4):
            if(not self.enviando_ruta):
                self.control_stade=0
                self.tomar_medida=False

if __name__ == "__main__":
    rospy.init_node("image_joke_py")

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)
    nodo=Detector_valle()
    rospy.spin()





