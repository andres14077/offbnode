#!/usr/bin/python -B
# -*- coding: utf-8 -*-
import rospy
from image_joke import Image_Joke
from offboard_node import Offboard_Master
from procesado_plc_2 import procesado_plc_2
from rute_plan import rute_plan
from depth_image_to_midas import Depth_image_to_midas
from detector_valle import Detector_valle
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger,Empty,EmptyResponse
import std_msgs.msg as std_msgs

class Maestro:
    def __init__(self):
        self.rate=rospy.Rate(20)
        self.accion_service=rospy.Service('offbnode/iniciar_toma', Empty, self.accion_cb)
        self.accion_service_2=rospy.Service('offbnode/iniciar_sin_reconocimiento', Empty, self.accion_2_cb)
        self.accion_service_3=rospy.Service('offbnode/iniciar_validacion_altura', Empty, self.accion_3_cb)
        self.identificar_terreno_client = rospy.ServiceProxy("offbnode/identificar_terreno", Trigger)
        self.tomar_medidas_terreno_client = rospy.ServiceProxy("offbnode/tomar_medidas_terreno", Trigger)
        self.tomar_medida_client = rospy.ServiceProxy("offbnode/tomar_medida", Trigger)
        self.calcular_ruta_client = rospy.ServiceProxy("offbnode/calcular_y_seguir_ruta", Trigger)
        self.Iniciar_Evaluacion_Altura_pub = rospy.Publisher("/evaluar_medida/start", std_msgs.Empty, queue_size=10)
        self.Terminar_Evaluacion_Altura_pub = rospy.Publisher("/evaluar_medida/fin", std_msgs.Empty, queue_size=10)
        self.Iniciar_Evaluacion_Altura_Stereo_pub = rospy.Publisher("/evaluar_medicion_altura/start", std_msgs.Empty, queue_size=10)
        self.Terminar_Evaluacion_Altura_Stereo_pub = rospy.Publisher("/evaluar_medicion_altura/fin", std_msgs.Empty, queue_size=10)
        self.set_pose_pub=rospy.Publisher("offbnode/set_point_to_measure_cmd", PoseStamped, queue_size=10)

    def accion_cb(self,req):
        tiempo_inicial=rospy.Time.now()
        self.identificar_terreno_client()
        self.tomar_medidas_terreno_client()
        tiempo_final=rospy.Time.now()
        duracion = tiempo_final - tiempo_inicial
        rospy.logwarn("Tiempo calculando ruta: %s seg",duracion.to_sec())
        self.Iniciar_Evaluacion_Altura_pub.publish()
        self.calcular_ruta_client()
        self.Terminar_Evaluacion_Altura_pub.publish()
        return EmptyResponse()

    def accion_2_cb(self,req):
        self.Iniciar_Evaluacion_Altura_pub.publish()
        self.calcular_ruta_client()
        self.Terminar_Evaluacion_Altura_pub.publish()
        return EmptyResponse()

    def accion_3_cb(self,req):
        punto_de_medida =PoseStamped()
        punto_de_medida.pose.position.z=50
        self.Iniciar_Evaluacion_Altura_Stereo_pub.publish()
        for i in range(1000):
            self.set_pose_pub.publish(punto_de_medida)
            self.tomar_medida_client()
            rospy.loginfo("Numero de tomas realizadas %d" % i)
        self.Terminar_Evaluacion_Altura_Stereo_pub.publish()
        return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node('maestro_nodo', anonymous=True)
    nodo1=Offboard_Master()
    nodo2=Image_Joke()
    nodo3=procesado_plc_2()
    nodo4=rute_plan()
    nodo5=Depth_image_to_midas()
    nodo6=Detector_valle()
    nodo=Maestro()
    while(not rospy.is_shutdown()):
        nodo1.update()
        nodo3.update()
        nodo.rate.sleep()
    #rospy.spin()

