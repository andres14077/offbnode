#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool,CommandBoolRequest
import numpy as np
import matplotlib.pyplot as plt

class Calculo_Variacion_Posicion:
    def __init__(self):
        #variables generales
        self.rate = rospy.Rate(10)
        self.current_local_pose=PoseStamped()
        self.set_pose=PoseStamped()
        plt.ion() # Activa el modo interactivo
        self.fig, self.ax = plt.subplots() # Crea la figura y los ejes
        self.xdata, self.ydata = [], [] # Listas para almacenar los datos
        self.ln, = plt.plot([], [], 'ro') # Inicializa una línea con puntos rojos

        #suscribtores y publicadores
        self.set_pose_sub=rospy.Subscriber("offbnode/pose_local_cmd", PoseStamped, self.set_pose_cb)
        self.local_pose_sub=rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_pose_cb)

        # sincronizacion con servicio
        rospy.logdebug("Esperando servicio offboard")
        rospy.wait_for_service("offbnode/master_ok")
        offboard_client = rospy.ServiceProxy("offbnode/master_ok", CommandBool)
        offboard_cmd = CommandBoolRequest()
        offboard_cmd.value = True
        while(offboard_client.call(offboard_cmd).success == False):
            self.rate.sleep()

    def local_pose_cb(self,msg):
        self.current_local_pose = msg
        x=self.distancia_to_setpoint(self.set_pose)
        self.xdata.append(x)
        self.ydata.append(0)
        self.ln.set_data(self.xdata, self.ydata)
        self.update_limits(self.xdata, self.ydata)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        rospy.loginfo(np.std(self.xdata))

    def set_pose_cb(self,msg):
        self.set_pose = msg

    def distancia_to_setpoint(self,pose):
        d= (self.current_local_pose.pose.position.x-pose.pose.position.x)*(self.current_local_pose.pose.position.x-pose.pose.position.x)
        d+=(self.current_local_pose.pose.position.y-pose.pose.position.y)*(self.current_local_pose.pose.position.y-pose.pose.position.y)
        d+=(self.current_local_pose.pose.position.z-pose.pose.position.z)*(self.current_local_pose.pose.position.z-pose.pose.position.z)
        return math.sqrt(d)

    def update_limits(self,x, y):
        self.ax.set_xlim(min(self.xdata)-1, max(self.xdata)+1)
        self.ax.set_ylim(min(self.ydata)-1, max(self.ydata)+1)

if __name__ == "__main__":
    rospy.init_node("image_joke_py")

    # Setpoint publishing MUST be faster than 2Hz
    nodo=Calculo_Variacion_Posicion()
    rospy.spin()
    # while(not rospy.is_shutdown()):
    #     nodo.update()
    #     nodo.rate.sleep()





