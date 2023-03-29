#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger,TriggerRequest,TriggerResponse


class Maestro:
    def __init__(self):
        self.rate=rospy.Rate(20)
        self.set_pose_pub=rospy.Publisher("offbnode/set_point_to_measure_cmd", PoseStamped, queue_size=10)
        rospy.wait_for_service("offbnode/tomar_medida")
        self.tomar_medida_client = rospy.ServiceProxy("offbnode/tomar_medida", Trigger)


if __name__ == '__main__':
    rospy.init_node('maestro_nodo', anonymous=True)
    serv=Maestro()
    rospy.spin()

