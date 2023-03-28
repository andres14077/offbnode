#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Bool


class Maestro:
    def __init__(self):


if __name__ == '__main__':
    rospy.init_node('maestro_nodo', anonymous=True)
    serv=Maestro()
    rate=rospy.Rate(20)
    rospy.spin()

