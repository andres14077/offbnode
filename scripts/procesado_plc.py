#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Bool
import os


class Service_kill:
    
    def __init__(self):
        rospy.loginfo("init node service kill ROS")
        rospy.Subscriber("/kill_ROS",Bool,self.Service_Calback)         
                                        
    def Service_Calback(self,msgg):
        if(msgg.data==True):
            os.system("kill $(ps -aux | grep offbnode| awk '{print $2}')")
            exit(0)
        
def servicio_de_kill():
    rospy.init_node('servicio_de_kill_node', anonymous=True)
    serv=Service_kill()
    rate=rospy.Rate(20)
    rospy.spin()
    
    
if __name__ == '__main__':
    try:
        servicio_de_kill()
    except rospy.ROSInterruptException:
        rospy.loginfo("Interrupcion")
        exit()
    except:
        rospy.loginfo("error")
        exit()

    
    
