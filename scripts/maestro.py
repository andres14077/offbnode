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
from std_srvs.srv import Trigger,Empty
import std_msgs.msg as std_msgs

class Maestro:
    def __init__(self):
        self.rate=rospy.Rate(40)
        self.accion_service=rospy.Service('offbnode/iniciar_toma', Empty, self.accion_cb)
        self.identificar_terreno_client = rospy.ServiceProxy("offbnode/identificar_terreno", Trigger)
        self.tomar_medidas_terreno_client = rospy.ServiceProxy("offbnode/tomar_medidas_terreno", Trigger)
        self.calcular_ruta_client = rospy.ServiceProxy("offbnode/calcular_y_seguir_ruta", Trigger)
        self.Iniciar_Evaluacion_Altura_pub = rospy.Publisher("/evaluar_medida/start", std_msgs.Empty, queue_size=10)
        self.Terminar_Evaluacion_Altura_pub = rospy.Publisher("/evaluar_medida/fin", std_msgs.Empty, queue_size=10)

    def accion_cb(self,req):
        self.identificar_terreno_client()
        self.tomar_medidas_terreno_client()
        self.Iniciar_Evaluacion_Altura_pub()
        self.calcular_ruta_client()
        self.Terminar_Evaluacion_Altura_pub()


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

