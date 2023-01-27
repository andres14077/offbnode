#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Vector3
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from mavros_msgs.msg import WaypointReached
from tf.transformations import quaternion_from_euler
import math

G_to_R=0.01745329251
class rute_plan:
    def __init__(self):
        self.Width_sensor=6.17                # en mm
        self.Height_sensor=4.63               # en mm
        self.Image_pix_Width=4608             # en pixeles
        self.Image_pix_Height=3456            # en pixeles
        self.Focal_length=4.9                 # en mm
        # configuracion de parametros de vuelo
        self.H=50                             # en m
        self.translape_lateral=70             # en %
        self.translape_longitudinal=70        # en %
        self.angulo_entrada=120               # en grados
        self.distancia_respuesta=20           # en m
        self.gradiente_x=0                    # en m
        self.gradiente_y=0                    # en m
        self.gradiente_z=0                    # en m
        # cerca de vuelo
        self.min_x=-50                        # en m desde el punto de partida
        self.min_y=-50                        # en m desde el punto de partida
        self.max_x=50                         # en m desde el punto de partida
        self.max_y=50                         # en m desde el punto de partida

        self.current_local_pose=PoseStamped()

        self.nav_pos_pub = rospy.Publisher("offboard/nav", Path,queue_size=10)
        self.cerca_pub = rospy.Publisher("offboard/cerca", PolygonStamped,queue_size=10)
        self.cerca_max_pub = rospy.Publisher("offboard/cerca_extra", PolygonStamped,queue_size=10)
        self.mensaje_camara_pub = rospy.Publisher("offboard/mission/reached", WaypointReached,queue_size=10)

        self.kill_ros_pub = rospy.Publisher("kill_ROS", Bool,queue_size=10)
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", 10)

        self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_pose_cb)


    def producto_cruz(self,v1,v2):
        v3 = Vector3()
        v3.x=v1.y*v2.z-v1.z*v2.y
        v3.y=v1.z*v2.x-v1.x*v2.z
        v3.z=v1.x*v2.y-v1.y*v2.x
        return v3
    def magnitud(self,v):
        a=v.x*v.x
        a+=v.y*v.y
        a+=v.z*v.z
        return math.sqrt(a)
    def distancia_punto_recta(self,v,pl,p):
        pl_p = Vector3()
        v1 = Vector3()
        pl_p.x=pl.x-p.x
        pl_p.y=pl.y-p.y
        pl_p.z=pl.z-p.z
        v1=self.producto_cruz(v,pl_p)
        return self.magnitud(v1)/self.magnitud(v)
    def distancia(self,l_x,l_y,l_z,s_x,s_y,s_z ):
        dx=(l_x-s_x)*(l_x-s_x)
        dy=(l_y-s_y)*(l_y-s_y)
        dz=(l_z-s_z)*(l_z-s_z)
        return math.sqrt(dx+dy+dz)

    def Dentro_de_Cerca(self,x_max, x_min, y_max, y_min, p):
        if(p.x<=x_max and p.x>=x_min):
            if(p.y<=y_max and p.y>=y_min):
                return True
            else:
                return False
        else:
            return False
    def angulo_en_rango(self, a):
        aa = 0
        if (a>180):
            aa= a-360
        elif (a<-180):
            aa= a+360
        else:
            aa= a
        if(aa==180):
            aa=179.9999
        elif(aa==90):
            aa=89.9999
        elif(aa==0):
            aa=0.0001
        elif(aa==-90):
            aa=-89.9999
        elif(aa==-180):
            aa=-179.9999
        return aa

    def Normalizar_vector(self,x,y,z):
        d=math.sqrt(x*x+y*y+z*z)
        v = Vector3()
        v.x=x/d
        v.y=y/d
        v.z=z/d
        return v

    def Recta(self,v,p,i):
        Punto_Recta = Point()
        Punto_Recta.x=v.x*i+p.x
        Punto_Recta.y=v.y*i+p.y
        Punto_Recta.z=p.z
        return Punto_Recta

    def x_en_recta(self, v, p, y ):
        Punto_Recta = Point()
        Punto_Recta.x=(y-p.y)*v.x/v.y+p.x
        Punto_Recta.y=y
        Punto_Recta.z=p.z
        return Punto_Recta

    def y_en_recta(self, v, p, x):
        Punto_Recta = Point()
        Punto_Recta.x=x
        Punto_Recta.y=(x-p.x)*v.y/v.x+p.y
        Punto_Recta.z=p.z
        return Punto_Recta

    def Plano_Z(self, v, p_plano, p_x_y ):
        Punto_Plano = Point()
        Punto_Plano.x=p_x_y.x
        Punto_Plano.y=p_x_y.y
        Punto_Plano.z=p_plano.z-(v.x*(p_x_y.x-p_plano.x)+v.y*(p_x_y.y-p_plano.y))/v.z
        return Punto_Plano

    def local_pose_cb(self,msg):
        self.current_local_pose = msg
    def ruta_plane_service_cb(self,req):
        self.angulo_entrada = req.data
        #calculo de variables de vuelo
        self.GSD=(self.Width_sensor*self.H*100)/(self.Focal_length*self.Image_pix_Width)             # en cm/pix
        self.ancho_huella=(self.GSD*self.Image_pix_Width)/100                              # en m
        self.alto_huella=(self.GSD*self.Image_pix_Height)/100                              # en m
        self.separacion_lineas_vuelo=self.ancho_huella*(1-(self.translape_lateral/100))    # en m
        self.base_en_aire=self.alto_huella*(1-(self.translape_longitudinal/100))           # en m

        self.min_cerca_x=self.min_x-self.distancia_respuesta
        self.min_cerca_y=self.min_y-self.distancia_respuesta
        self.max_cerca_x=self.max_x+self.distancia_respuesta
        self.max_cerca_y=self.max_y+self.distancia_respuesta

        rospy.loginfo("Parametros de vuelo:\n-Configuracion de camara:\n  Ancho del sensor        = %.3f mm\n  Altura del sensor       = %.3f mm\n  Pixeles por ancho       = %.0f pix\n  Pixeles por alto        = %.0f pix\n  Longitud focal          = %.2f mm\n-Configuracion de vuelo:\n  Altura de vuelo         = %.2f m\n  GSD                     = %.2f cm/pix\n  Translape lateral       = %.2f %%\n  Translape longitudinal  = %.2f %%\n  Angulo entrada          = %.2f \n",
            self.Width_sensor,self.self.Height_sensor,self.Image_pix_Width,self.Image_pix_Height,
            self.Focal_length,self.H,self.GSD,self.translape_lateral,self.translape_longitudinal,
            self.angulo_entrada)
        ###/cercas de vuelo 
        cerca = PolygonStamped()
        cerca_max = PolygonStamped()

        cerca.header.stamp=rospy.Time.now()
        cerca.header.frame_id="map"

        p1 =Point32()

        p1.x=self.max_x
        p1.y=self.max_y
        p1.z=self.H
        cerca.polygon.points.append(p1)
        p1.x=self.max_x
        p1.y=self.min_y
        p1.z=self.H
        cerca.polygon.points.append(p1)
        p1.x=self.min_x
        p1.y=self.min_y
        p1.z=self.H
        cerca.polygon.points.append(p1)
        p1.x=self.min_x
        p1.y=self.max_y
        p1.z=self.H
        cerca.polygon.points.append(p1)

        cerca_max.header.stamp=ros::Time::now()
        cerca_max.header.frame_id="map"

        p1.x=self.max_cerca_x
        p1.y=self.max_cerca_y
        p1.z=self.H
        cerca_max.polygon.points.append(p1)
        p1.x=self.max_cerca_x
        p1.y=self.min_cerca_y
        p1.z=self.H
        cerca_max.polygon.points.append(p1)
        p1.x=self.min_cerca_x
        p1.y=self.min_cerca_y
        p1.z=self.H
        cerca_max.polygon.points.append(p1)
        p1.x=self.min_cerca_x
        p1.y=self.max_cerca_y
        p1.z=self.H
        cerca_max.polygon.points.append(p1)

        vector_linea_y_max = Vector3()
        punto_linea_y_max = Point()
        vector_linea_y_max.x=1
        punto_linea_y_max.y=self.max_cerca_y
        punto_linea_y_max.z=self.H

        vector_linea_y_min = Vector3()
        punto_linea_y_min = Point()
        vector_linea_y_min.x=1
        punto_linea_y_min.y=self.min_cerca_y
        punto_linea_y_min.z=self.H

        vector_linea_x_max = Vector3()
        punto_linea_x_max = Point()
        vector_linea_x_max.y=1
        punto_linea_x_max.x=self.max_cerca_x
        punto_linea_x_max.z=self.H

        vector_linea_x_min = Vector3()
        punto_linea_x_min = Point()
        vector_linea_x_min.y=1
        punto_linea_x_min.x=self.min_cerca_x
        punto_linea_x_min.z=self.H
    ### calculo de lineas de vuelo planas

        angulo_entrada=angulo_en_rango(angulo_entrada)
        geometry_msgs::Vector3 vector_avance=Normalizar_vector(cos(angulo_entrada*G_to_R),sin(angulo_entrada*G_to_R),0)
        vector_avance.x=vector_avance.x*base_en_aire
        vector_avance.y=vector_avance.y*base_en_aire
        geometry_msgs::Point punto_arranque,dis_entre_lineas
        dis_entre_lineas.x=separacion_lineas_vuelo*sin(angulo_entrada*G_to_R)
        dis_entre_lineas.y=separacion_lineas_vuelo*cos(angulo_entrada*G_to_R)
        #punto de arranque
        if(angulo_entrada<180 and angulo_entrada>90):
            ROS_INFO("ang1")
            dis_entre_lineas.x*=-1
            dis_entre_lineas.y*=-1
            punto_arranque.x= max_cerca_x
            punto_arranque.y= max_cerca_y
            punto_arranque.z= H
            punto_arranque.x= punto_arranque.x+dis_entre_lineas.x
            punto_arranque.y= punto_arranque.y-dis_entre_lineas.y
            punto_arranque=y_en_recta(vector_avance,punto_arranque,max_cerca_x)
            if(not Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                punto_arranque=x_en_recta(vector_avance,punto_arranque,min_cerca_y)
            
        elif(angulo_entrada<90 and angulo_entrada>0):
            ROS_INFO("ang2")
            punto_arranque.x= min_cerca_x
            punto_arranque.y= max_cerca_y
            punto_arranque.z= H
            punto_arranque.x= punto_arranque.x+dis_entre_lineas.x
            punto_arranque.y= punto_arranque.y-dis_entre_lineas.y
            punto_arranque=y_en_recta(vector_avance,punto_arranque,min_cerca_x)
            if(not Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                punto_arranque=x_en_recta(vector_avance,punto_arranque,min_cerca_y)
            
        elif(angulo_entrada<0 and angulo_entrada>-90):
            ROS_INFO("ang3")
            dis_entre_lineas.x*=-1
            dis_entre_lineas.y*=-1
            punto_arranque.x= min_cerca_x
            punto_arranque.y= min_cerca_y
            punto_arranque.z= H
            punto_arranque.x= punto_arranque.x+dis_entre_lineas.x
            punto_arranque.y= punto_arranque.y-dis_entre_lineas.y
            punto_arranque=y_en_recta(vector_avance,punto_arranque,min_cerca_x)
            if(not Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                punto_arranque=x_en_recta(vector_avance,punto_arranque,max_cerca_y)
            
        elif(angulo_entrada<-90 and angulo_entrada>-180):
            ROS_INFO("ang4")
            punto_arranque.x= max_cerca_x
            punto_arranque.y= min_cerca_y
            punto_arranque.z= H
            punto_arranque.x= punto_arranque.x+dis_entre_lineas.x
            punto_arranque.y= punto_arranque.y-dis_entre_lineas.y
            punto_arranque=y_en_recta(vector_avance,punto_arranque,max_cerca_x)
            if(not Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                punto_arranque=x_en_recta(vector_avance,punto_arranque,max_cerca_y)
            
        

        nav_msgs::Path path
        path.header.stamp = ros::Time::now()
        path.header.frame_id= "map"

        ROS_INFO("path")
        for (int i = 0 i < 1000 ++i):
            geometry_msgs::PoseStamped pose
            pose.header.stamp = ros::Time::now()
            pose.header.frame_id = "map"

            geometry_msgs::Point px=Recta(vector_avance,punto_arranque,i)
            if(not Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,px)):
                px=Recta(vector_avance,punto_arranque,i-1)
                double d1,d2,d3,d4
                d1=distancia_punto_recta(vector_linea_x_max,punto_linea_x_max,px)
                d2=distancia_punto_recta(vector_linea_x_min,punto_linea_x_min,px)
                d3=distancia_punto_recta(vector_linea_y_max,punto_linea_y_max,px)
                d4=distancia_punto_recta(vector_linea_y_min,punto_linea_y_min,px)
                # cambio a siguiente recta
                punto_arranque.x= punto_arranque.x+dis_entre_lineas.x
                punto_arranque.y= punto_arranque.y-dis_entre_lineas.y
                vector_avance.x=vector_avance.x*(-1)
                vector_avance.y=vector_avance.y*(-1)
                if( d1<=d2 and d1<=d3 and d1<=d4):
                    punto_arranque=y_en_recta(vector_avance,punto_arranque,max_cerca_x)
                    if(not Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                        if(d3<d4):
                            punto_arranque=x_en_recta(vector_avance,punto_arranque,max_cerca_y)
                        else:
                            punto_arranque=x_en_recta(vector_avance,punto_arranque,min_cerca_y)
                        
                    
                elif( d2<=d3 and d2<=d4 ):
                    punto_arranque=y_en_recta(vector_avance,punto_arranque,min_cerca_x)
                    if(not Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                        if(d3<d4):
                            punto_arranque=x_en_recta(vector_avance,punto_arranque,max_cerca_y)
                        else:
                            punto_arranque=x_en_recta(vector_avance,punto_arranque,min_cerca_y)
                        
                    
                elif( d3<=d4 ):
                    punto_arranque=x_en_recta(vector_avance,punto_arranque,max_cerca_y)
                    if(not Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                        if(d1<d2):
                            punto_arranque=y_en_recta(vector_avance,punto_arranque,max_cerca_x)
                        else:
                            punto_arranque=y_en_recta(vector_avance,punto_arranque,min_cerca_x)
                        
                    
                else:
                    punto_arranque=x_en_recta(vector_avance,punto_arranque,min_cerca_y)
                    if(not Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                        if(d1<d2):
                            punto_arranque=y_en_recta(vector_avance,punto_arranque,max_cerca_x)
                        else:
                            punto_arranque=y_en_recta(vector_avance,punto_arranque,min_cerca_x)
                        
                    
                
                i=0
                px=Recta(vector_avance,punto_arranque,i)
                if(not Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                    i=1000
                    px.x=0
                    px.y=0
                
            
            #ROS_INFO("punto en recta x=[%f],y=[%f],z=[%f]",px.x, px.y, px.z)
            pose.pose.position.x = px.x
            pose.pose.position.y = px.y
            pose.pose.position.z = px.z
            pose.pose.orientation=tf::createQuaternionMsgFromYaw(atan2(vector_avance.y,vector_avance.x))
            path.poses.append(pose)
        
        for (int i = 0 i < 5 ++i) :
            nav_pos_pub.publish(path)
            cerca_pub.publish(cerca)
            cerca_max_pub.publish(cerca_max)
            ros::spinOnce()
            rate.sleep()

if __name__ == '__main__':

    rospy.init_node('rute_square', anonymous=True)
    rate=rospy.Rate(20)
    nodo=rute_plan()
    rospy.spin()




