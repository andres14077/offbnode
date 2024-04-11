#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from mavros_msgs.msg import WaypointReached
from std_msgs.msg import Float64
from std_msgs.msg import Empty
import numpy as np


class EvaluarRecorrido:

    def __init__(self):
        rospy.loginfo("init node evaluar recorrido for offboard")
        rospy.Subscriber("/offbnode/mission/reached",WaypointReached,self.Waypoint_Reached_Calback)
        rospy.Subscriber("/iris_gimbal/distance_to_ground", Float64, self.Distance_Calback)
        rospy.Subscriber("/evaluar_medida/start", Empty, self.Iniciar_cb)
        rospy.Subscriber("/evaluar_medida/fin", Empty, self.Terminar_cb)
        self.distancia=0
        self.tomas=[]

    def Distance_Calback(self,msg):
        self.distancia=msg.data

    def Waypoint_Reached_Calback(self,waypoint):
        self.tomas.append(self.distancia)
        # rospy.loginfo(self.tomas)

    def Iniciar_cb(self,msg):
        self.tomas=[]
        self.tiempo_inicial=rospy.Time.now()

    def Terminar_cb(self,msg):
        self.tiempo_final=rospy.Time.now()
        duracion = self.tiempo_final - self.tiempo_inicial
        H = rospy.get_param("/maestro/Altura_vuelo")
        Width_sensor = rospy.get_param("/maestro/Ancho_sensor")
        Focal_length = rospy.get_param("/maestro/Longitud_focal")
        Image_pix_Width = rospy.get_param("/maestro/Ancho_imagen_pix")
        factor_GSD=(Width_sensor *100.0)/(Focal_length * Image_pix_Width)
        error=np.sqrt(np.mean(([(n - H)**2 for n in self.tomas])))
        rospy.logwarn("Error cuadratico medio de altura: %f m",error)
        rospy.logwarn("desviacion estandar de altura: %f m",np.std(self.tomas))
        rospy.logwarn("Promedio de altura: %f m",np.mean(self.tomas) )
        rospy.logwarn("Error cuadratico medio de GSD: %f cm/pix",error * factor_GSD)
        rospy.logwarn("desviacion estandar de GSD: %f cm/pix",np.std(self.tomas) * factor_GSD)
        rospy.logwarn("Promedio de GSD: %f cm/pix",np.mean(self.tomas) * factor_GSD)
        rospy.logwarn("tiempo recorrido de vuelo: %s seg",duracion.to_sec())
        self.tomas=[]


if __name__ == '__main__':
    rospy.init_node('evaluar_recorrido', anonymous=True)
    cap=EvaluarRecorrido()
    rate=rospy.Rate(1)
    rospy.spin()



