#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import copy
import math
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from std_srvs.srv import Trigger,TriggerResponse
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped,Vector3
from geometry_msgs.msg import PointStamped,Point,Point32
from geometry_msgs.msg import PolygonStamped
from offbnode.msg import PlaneStamped
from mavros_msgs.msg import WaypointReached,MountControl
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from dynamic_reconfigure.server import Server
from offbnode.cfg import rute_planConfig

G_to_R=math.pi/180.0
R_to_G=180.0/math.pi
class rute_plan:
    def __init__(self):

        self.rate=rospy.Rate(2)

        self.current_local_pose=PoseStamped()
        self.plane_in_map = PlaneStamped()
        self.plane_in_map.vector.vector.z = 1
        self.planos_individuales=[]
        self.is_valley=False

        self.nav_pos_pub = rospy.Publisher("offbnode/nav", Path,queue_size=10)
        self.nav_z_pos_pub = rospy.Publisher("offbnode/nav_z", Path,queue_size=10)
        self.cerca_pub = rospy.Publisher("offbnode/cerca", PolygonStamped,queue_size=10)
        self.cerca_max_pub = rospy.Publisher("offbnode/cerca_extra", PolygonStamped,queue_size=10)

        self.local_poss_pub = rospy.Publisher("offbnode/pose_local_cmd", PoseStamped, queue_size=10)
        self.camera_pose_pub=rospy.Publisher('mavros/mount_control/command', MountControl, queue_size=10)
        self.mensaje_camara_pub = rospy.Publisher("offbnode/mission/reached", WaypointReached,queue_size=10)

        self.kill_ros_pub = rospy.Publisher("kill_ROS", Bool,queue_size=10)

        self.local_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_pose_cb)
        self.point_sub=rospy.Subscriber('offbnode/plano_promedio_in_map', PlaneStamped, self.plane_in_map_cb)
        self.is_valle_sub=rospy.Subscriber('offbnode/is_valle', Bool, self.is_valle_cb)
        self.plane_sub=rospy.Subscriber('offbnode/plane_individual_in_map', PlaneStamped, self.plane_individual_in_map_cb)

        self.calcular_ruta_service = rospy.Service("offbnode/calcular_y_seguir_ruta", Trigger,self.ruta_plane_service_cb)
        self.reconfigure_params_server = Server(rute_planConfig, self.reconfigure_params_cb)

        rospy.loginfo("Rute plan init")

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

    def distancia(self,p1,p2):
        dx=(p1.x-p2.x)*(p1.x-p2.x)
        dy=(p1.y-p2.y)*(p1.y-p2.y)
        dz=(p1.z-p2.z)*(p1.z-p2.z)
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

    def Plano_Z(self,p_x_y ):
        Punto_Plano = Point()
        Punto_Plano.x=p_x_y.x
        Punto_Plano.y=p_x_y.y
        if(self.is_valle):
            j=[]
            for i in self.planos_individuales:
                v=i.vector.vector
                p=i.point.point
                j.append(p.z-(v.x*(p_x_y.x-p.x)+v.y*(p_x_y.y-p.y))/v.z + self.H)
            Punto_Plano.z=max(j)
        else:
            v=self.plane_in_map.vector.vector
            p=self.plane_in_map.point.point
            Punto_Plano.z=p.z-(v.x*(p_x_y.x-p.x)+v.y*(p_x_y.y-p.y))/v.z + self.H
        return Punto_Plano

    def calcular_ruta(self):
        if(self.angulo_automatico and self.plane_in_map.vector.vector.x !=0 and self.plane_in_map.vector.vector.y !=0):
            angulo_entrada = math.atan2(self.plane_in_map.vector.vector.y,self.plane_in_map.vector.vector.x)
            angulo_entrada *= R_to_G
            angulo_entrada += 90
        else:
            angulo_entrada = self.angulo_entrada
        #calculo de variables de vuelo
        GSD=(self.Width_sensor*self.H*100.0)/(self.Focal_length * self.Image_pix_Width)             # en cm/pix
        ancho_huella=(GSD*self.Image_pix_Width)/100.0                              # en m
        alto_huella=(GSD*self.Image_pix_Height)/100.0                              # en m
        separacion_lineas_vuelo= ancho_huella * (1.0 - (self.translape_lateral / 100.0 ))    # en m
        base_en_aire=alto_huella*(1.0-(self.translape_longitudinal/100.0))           # en m

        min_cerca_x=self.min_x-self.distancia_respuesta
        min_cerca_y=self.min_y-self.distancia_respuesta
        max_cerca_x=self.max_x+self.distancia_respuesta
        max_cerca_y=self.max_y+self.distancia_respuesta

        rospy.loginfo("Parametros de vuelo:\n-Configuracion de camara:\n  Ancho del sensor        = %.3f mm\n  Altura del sensor       = %.3f mm\n  Pixeles por ancho       = %.0f pix\n  Pixeles por alto        = %.0f pix\n  Longitud focal          = %.2f mm\n-Configuracion de vuelo:\n  Altura de vuelo         = %.2f m\n  GSD                     = %.2f cm/pix\n  Translape lateral       = %.2f %%\n  Translape longitudinal  = %.2f %%\n  Angulo entrada          = %.2f \n",
                    self.Width_sensor,self.Height_sensor,self.Image_pix_Width,self.Image_pix_Height,
                    self.Focal_length,self.H,GSD,self.translape_lateral,self.translape_longitudinal,
                    angulo_entrada)
        ###/cercas de vuelo 
        cerca = PolygonStamped()
        cerca_max = PolygonStamped()

        cerca.header.stamp=rospy.Time.now()
        cerca.header.frame_id="map"

        p1 =Point32()

        p1.x=self.max_x
        p1.y=self.max_y
        p1.z=self.H
        cerca.polygon.points.append(copy.deepcopy(p1))
        p1.x=self.max_x
        p1.y=self.min_y
        p1.z=self.H
        cerca.polygon.points.append(copy.deepcopy(p1))
        p1.x=self.min_x
        p1.y=self.min_y
        p1.z=self.H
        cerca.polygon.points.append(copy.deepcopy(p1))
        p1.x=self.min_x
        p1.y=self.max_y
        p1.z=self.H
        cerca.polygon.points.append(copy.deepcopy(p1))

        cerca_max.header.stamp=rospy.Time.now()
        cerca_max.header.frame_id="map"

        p1.x=max_cerca_x
        p1.y=max_cerca_y
        p1.z=self.H
        cerca_max.polygon.points.append(copy.deepcopy(p1))
        p1.x=max_cerca_x
        p1.y=min_cerca_y
        p1.z=self.H
        cerca_max.polygon.points.append(copy.deepcopy(p1))
        p1.x=min_cerca_x
        p1.y=min_cerca_y
        p1.z=self.H
        cerca_max.polygon.points.append(copy.deepcopy(p1))
        p1.x=min_cerca_x
        p1.y=max_cerca_y
        p1.z=self.H
        cerca_max.polygon.points.append(copy.deepcopy(p1))

        vector_linea_y_max = Vector3()
        punto_linea_y_max = Point()
        vector_linea_y_max.x=1.0
        punto_linea_y_max.y=max_cerca_y
        punto_linea_y_max.z=self.H

        vector_linea_y_min = Vector3()
        punto_linea_y_min = Point()
        vector_linea_y_min.x=1.0
        punto_linea_y_min.y=min_cerca_y
        punto_linea_y_min.z=self.H

        vector_linea_x_max = Vector3()
        punto_linea_x_max = Point()
        vector_linea_x_max.y=1.0
        punto_linea_x_max.x=max_cerca_x
        punto_linea_x_max.z=self.H

        vector_linea_x_min = Vector3()
        punto_linea_x_min = Point()
        vector_linea_x_min.y=1.0
        punto_linea_x_min.x=min_cerca_x
        punto_linea_x_min.z=self.H
    ### calculo de lineas de vuelo planas

        angulo_entrada=self.angulo_en_rango(angulo_entrada)
        vector_avance=self.Normalizar_vector(math.cos(angulo_entrada*G_to_R),math.sin(angulo_entrada*G_to_R),0)
        vector_avance.x=vector_avance.x*base_en_aire
        vector_avance.y=vector_avance.y*base_en_aire
        punto_arranque = Point()
        dis_entre_lineas = Point()
        dis_entre_lineas.x=separacion_lineas_vuelo*math.sin(angulo_entrada*G_to_R)
        dis_entre_lineas.y=separacion_lineas_vuelo*math.cos(angulo_entrada*G_to_R)
        #punto de arranque
        if(angulo_entrada<180 and angulo_entrada>90):
            rospy.loginfo("ang1")
            dis_entre_lineas.x*=-1.0
            dis_entre_lineas.y*=-1.0
            punto_arranque.x= max_cerca_x
            punto_arranque.y= max_cerca_y
            punto_arranque.z= self.H
            punto_arranque.x= punto_arranque.x+dis_entre_lineas.x
            punto_arranque.y= punto_arranque.y-dis_entre_lineas.y
            punto_arranque=self.y_en_recta(vector_avance,punto_arranque,max_cerca_x)
            if(not self.Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                punto_arranque=self.x_en_recta(vector_avance,punto_arranque,min_cerca_y)
        elif(angulo_entrada<90 and angulo_entrada>0):
            rospy.loginfo("ang2")
            punto_arranque.x= min_cerca_x
            punto_arranque.y= max_cerca_y
            punto_arranque.z= self.H
            punto_arranque.x= punto_arranque.x+dis_entre_lineas.x
            punto_arranque.y= punto_arranque.y-dis_entre_lineas.y
            punto_arranque=self.y_en_recta(vector_avance,punto_arranque,min_cerca_x)
            if(not self.Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                punto_arranque=self.x_en_recta(vector_avance,punto_arranque,min_cerca_y)
        elif(angulo_entrada<0 and angulo_entrada>-90):
            rospy.loginfo("ang3")
            dis_entre_lineas.x*=-1.0
            dis_entre_lineas.y*=-1.0
            punto_arranque.x= min_cerca_x
            punto_arranque.y= min_cerca_y
            punto_arranque.z= self.H
            punto_arranque.x= punto_arranque.x+dis_entre_lineas.x
            punto_arranque.y= punto_arranque.y-dis_entre_lineas.y
            punto_arranque=self.y_en_recta(vector_avance,punto_arranque,min_cerca_x)
            if(not self.Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                punto_arranque=self.x_en_recta(vector_avance,punto_arranque,max_cerca_y)
        elif(angulo_entrada<-90 and angulo_entrada>-180):
            rospy.loginfo("ang4")
            punto_arranque.x= max_cerca_x
            punto_arranque.y= min_cerca_y
            punto_arranque.z= self.H
            punto_arranque.x= punto_arranque.x+dis_entre_lineas.x
            punto_arranque.y= punto_arranque.y-dis_entre_lineas.y
            punto_arranque=self.y_en_recta(vector_avance,punto_arranque,max_cerca_x)
            if(not self.Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                punto_arranque=self.x_en_recta(vector_avance,punto_arranque,max_cerca_y)

        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id= "map"
        path_z = Path()
        path_z.header.stamp = rospy.Time.now()
        path_z.header.frame_id= "map"

        rospy.loginfo("path")
        i=0
        while (i < 1000):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"

            px=self.Recta(vector_avance,punto_arranque,i)
            if(not self.Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,px)):
                px=self.Recta(vector_avance,punto_arranque,i-1)
                d1=self.distancia_punto_recta(vector_linea_x_max,punto_linea_x_max,px)
                d2=self.distancia_punto_recta(vector_linea_x_min,punto_linea_x_min,px)
                d3=self.distancia_punto_recta(vector_linea_y_max,punto_linea_y_max,px)
                d4=self.distancia_punto_recta(vector_linea_y_min,punto_linea_y_min,px)
                # cambio a siguiente recta
                punto_arranque.x= punto_arranque.x+dis_entre_lineas.x
                punto_arranque.y= punto_arranque.y-dis_entre_lineas.y
                vector_avance.x=vector_avance.x*(-1.0)
                vector_avance.y=vector_avance.y*(-1.0)
                if( d1<=d2 and d1<=d3 and d1<=d4):
                    punto_arranque=self.y_en_recta(vector_avance,punto_arranque,max_cerca_x)
                    if(not self.Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                        if(d3<d4):
                            punto_arranque=self.x_en_recta(vector_avance,punto_arranque,max_cerca_y)
                        else:
                            punto_arranque=self.x_en_recta(vector_avance,punto_arranque,min_cerca_y)
                elif( d2<=d3 and d2<=d4 ):
                    punto_arranque=self.y_en_recta(vector_avance,punto_arranque,min_cerca_x)
                    if(not self.Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                        if(d3<d4):
                            punto_arranque=self.x_en_recta(vector_avance,punto_arranque,max_cerca_y)
                        else:
                            punto_arranque=self.x_en_recta(vector_avance,punto_arranque,min_cerca_y)
                elif( d3<=d4 ):
                    punto_arranque=self.x_en_recta(vector_avance,punto_arranque,max_cerca_y)
                    if(not self.Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                        if(d1<d2):
                            punto_arranque=self.y_en_recta(vector_avance,punto_arranque,max_cerca_x)
                        else:
                            punto_arranque=self.y_en_recta(vector_avance,punto_arranque,min_cerca_x)
                else:
                    punto_arranque=self.x_en_recta(vector_avance,punto_arranque,min_cerca_y)
                    if(not self.Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                        if(d1<d2):
                            punto_arranque=self.y_en_recta(vector_avance,punto_arranque,max_cerca_x)
                        else:
                            punto_arranque=self.y_en_recta(vector_avance,punto_arranque,min_cerca_x)
                i=0
                px=self.Recta(vector_avance,punto_arranque,i)
                if(not self.Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                    i=1000
                    px.x=0.0
                    px.y=0.0
            # rospy.loginfo("punto en recta x=[%f],y=[%f],z=[%f],i=%d",px.x, px.y, px.z,i)
            pose.pose.position.x = px.x
            pose.pose.position.y = px.y
            pose.pose.position.z = px.z
            q=quaternion_from_euler(0,0,math.atan2(vector_avance.y,vector_avance.x))
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            path.poses.append(copy.deepcopy(pose))
            pose.pose.position=self.Plano_Z(px)
            path_z.poses.append(copy.deepcopy(pose))
            i+=1

        self.path_z=path_z
        self.nav_pos_pub.publish(path)
        self.nav_z_pos_pub.publish(path_z)
        self.cerca_pub.publish(cerca)
        self.cerca_max_pub.publish(cerca_max)

    def is_valle_cb(self,msg):
        self.is_valle=msg.data

    def plane_individual_in_map_cb(self,msg):
        self.planos_individuales.append(copy.deepcopy(msg))

    def local_pose_cb(self,msg):
        self.current_local_pose = msg

    def plane_in_map_cb(self,msg):
        self.plane_in_map = msg

    def reconfigure_params_cb(self,config, level):

        self.Width_sensor=config['Ancho_sensor']                    # en mm
        self.Height_sensor=config['Altura_sensor']                  # en mm
        self.Image_pix_Width=config['Ancho_imagen_pix']             # en pixeles
        self.Image_pix_Height=config['Altura_imagen_pix']           # en pixeles
        self.Focal_length=config['Longitud_focal']                  # en mm
        # configuracion de parametros de vuelo
        self.H=config['Altura_vuelo']                               # en m
        self.translape_lateral=config['Translape_lateral']          # en %
        self.translape_longitudinal=config['Translape_longitudinal']# en %
        self.angulo_automatico=config['Angulo_automatico']          # boolean
        self.angulo_entrada=config['Angulo_entrada']                # en grados
        self.distancia_respuesta=config['Turnaround_dist']          # en m
        # cerca de vuelo
        self.min_x=config['min_x']                                  # en m desde el punto de partida
        self.min_y=config['min_y']                                  # en m desde el punto de partida
        self.max_x=config['max_x']                                  # en m desde el punto de partida
        self.max_y=config['max_y']                                  # en m desde el punto de partida
        return config

    def ruta_plane_service_cb(self,req):
        self.calcular_ruta()
        last_view_porcentaje = rospy.Time.now()
        i=0
        while (i< len(self.path_z.poses)):
            pose=self.path_z.poses[i]
            #Calcular orientacion de camara
            q = pose.pose.orientation
            orientation_list = [q.x, q.y, q.z, q.w]
            angulos= euler_from_quaternion (orientation_list)
            camera_pose=MountControl()
            camera_pose.header.frame_id=""
            camera_pose.mode=2
            camera_pose.pitch=-90
            camera_pose.roll=0
            camera_pose.yaw=angulos[2] * -R_to_G
            #enviar posicion de camera
            self.local_poss_pub.publish(pose)
            self.camera_pose_pub.publish(camera_pose)
            #Revisar alcanse de objetivo
            if(self.distancia(pose.pose.position,self.current_local_pose.pose.position)<0.5):
                i+=1
                tomar_foto = WaypointReached()
                tomar_foto.header.frame_id = "map"
                tomar_foto.header.stamp = rospy.Time.now()
                tomar_foto.wp_seq=i
                self.mensaje_camara_pub.publish(tomar_foto)
            if(rospy.Time.now() - last_view_porcentaje > rospy.Duration(5.0)):
                rospy.loginfo("Porcentade de mision %f%%",(i*100.0/len(self.path_z.poses)))
                last_view_porcentaje = rospy.Time.now()
            self.rate.sleep()
        response=TriggerResponse()
        response.success=True
        return response


if __name__ == '__main__':

    rospy.init_node('rute_square', anonymous=True)
    nodo=rute_plan()
    rospy.spin()




