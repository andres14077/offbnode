#!/usr/bin/python3 -B
# -*- coding: utf-8 -*-
import rospy
from midas_tf import Midas_tf
from procesado_plc import procesado_plc


class Esclavo_3:
    def __init__(self):
        self.rate=rospy.Rate(40)

if __name__ == '__main__':
    rospy.init_node('esclavo_3_nodo', anonymous=True)
    nodo1=Midas_tf()
    nodo2=procesado_plc()
    nodo=Esclavo_3()
    rospy.spin()

