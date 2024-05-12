#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from offbnode.msg import PlaneStamped
import numpy as np


class EvaluarMedicionAltura:

    def __init__(self):
        rospy.loginfo("init node evaluar recorrido for offboard")
        rospy.Subscriber("/iris_gimbal/distance_to_ground", Float64, self.Distance_Calback)
        rospy.Subscriber("/evaluar_medicion_altura/start", Empty, self.Iniciar_cb)
        rospy.Subscriber("/evaluar_medicion_altura/fin", Empty, self.Terminar_cb)
        rospy.Subscriber('offbnode/plano_in_local', PlaneStamped, self.Plano_Medido_Callback )
        self.distancia=0
        self.tomas=[]

    def Distance_Calback(self,msg):
        self.distancia=msg.data

    def Iniciar_cb(self,msg):
        self.tomas=[]

    def Plano_Medido_Callback(self,msg):
        self.tomas.append([self.distancia,msg.point.point.z])

    def Terminar_cb(self,msg):
        rospy.logwarn(self.tomas)
        error=np.sqrt(np.mean(([(Hm - Hr)**2 for (Hr,Hm) in self.tomas])))
        error_porcentual= np.sqrt(np.mean(([((Hm - Hr)/Hr)**2 for (Hr,Hm) in self.tomas])))*100
        tomas_array = np.array(self.tomas)
        # rospy.logwarn("factor_GSD: %10.20f",factor_GSD)
        rospy.logwarn("RMSE altura: %f m",error)
        rospy.logwarn("RMSPE altura: %f %%", error_porcentual)
        rospy.logwarn("desviacion estandar altura: %f m",np.std(tomas_array[:,1]))
        rospy.logwarn("Promedio de altura: %f m",np.mean(tomas_array[:,1]))
        self.tomas=[]


if __name__ == '__main__':
    rospy.init_node('evaluar_recorrido', anonymous=True)
    cap=EvaluarMedicionAltura()
    rate=rospy.Rate(1)
    rospy.spin()



