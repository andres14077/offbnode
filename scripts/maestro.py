#!/usr/bin/python -B
# -*- coding: utf-8 -*-
import rospy
from image_joke import Image_Joke
from offboard_node import Offboard_Master
from procesado_plc_2 import procesado_plc_2
from rute_plan import rute_plan
from depth_image_to_midas import Depth_image_to_midas
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger,TriggerRequest,TriggerResponse
from std_msgs.msg import Bool


class Maestro:
    def __init__(self):
        self.rate=rospy.Rate(40)
        self.set_pose_pub=rospy.Publisher("offbnode/set_point_to_measure_cmd", PoseStamped, queue_size=10)
        self.accion_sub=rospy.Subscriber('offbnode/iniciar_toma', Bool, self.accion_cb)
        rospy.wait_for_service("offbnode/tomar_medida")
        self.tomar_medida_client = rospy.ServiceProxy("offbnode/tomar_medida", Trigger)
    def accion_cb(self,msg):
        punto_de_medida =PoseStamped()
        punto_de_medida.pose.position.x=0
        punto_de_medida.pose.position.y=0
        punto_de_medida.pose.position.z=50
        self.set_pose_pub.publish(punto_de_medida)
        self.tomar_medida_client()
        punto_de_medida.pose.position.x=40
        punto_de_medida.pose.position.y=40
        punto_de_medida.pose.position.z=50
        self.set_pose_pub.publish(punto_de_medida)
        self.tomar_medida_client()
        punto_de_medida.pose.position.x=-40
        punto_de_medida.pose.position.y=40
        punto_de_medida.pose.position.z=50
        self.set_pose_pub.publish(punto_de_medida)
        self.tomar_medida_client()
        punto_de_medida.pose.position.x=40
        punto_de_medida.pose.position.y=-40
        punto_de_medida.pose.position.z=50
        self.set_pose_pub.publish(punto_de_medida)
        self.tomar_medida_client()
        punto_de_medida.pose.position.x=-40
        punto_de_medida.pose.position.y=-40
        punto_de_medida.pose.position.z=50
        self.set_pose_pub.publish(punto_de_medida)
        self.tomar_medida_client()


if __name__ == '__main__':
    rospy.init_node('maestro_nodo', anonymous=True)
    nodo1=Offboard_Master()
    nodo2=Image_Joke()
    nodo3=procesado_plc_2()
    nodo4=rute_plan()
    nodo5=Depth_image_to_midas()
    nodo=Maestro()
    while(not rospy.is_shutdown()):
        nodo1.update()
        nodo3.update()
        nodo.rate.sleep()
    #rospy.spin()

