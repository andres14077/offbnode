
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


if __name__ == '__main__':
    angulo_entrada = atof(argv[1])
    #calculo de variables de vuelo
    double GSD=(Width_sensor*H*100)/(Focal_length*Image_pix_Width)             # en cm/pix
    double ancho_huella=(GSD*Image_pix_Width)/100                              # en m
    double alto_huella=(GSD*Image_pix_Height)/100                              # en m
    double separacion_lineas_vuelo=ancho_huella*(1-(translape_lateral/100))    # en m
    double base_en_aire=alto_huella*(1-(translape_longitudinal/100))           # en m

    double min_cerca_x=min_x-distancia_respuesta
    double min_cerca_y=min_y-distancia_respuesta
    double max_cerca_x=max_x+distancia_respuesta
    double max_cerca_y=max_y+distancia_respuesta


    ros::init(argc, argv, "offb_node_rute_square")
    ros::NodeHandle nh

    #the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0)


    ros::Publisher nav_pos_pub = nh.advertise<nav_msgs::Path>
            ("offboard/nav", 10)
    ros::Publisher cerca_pub = nh.advertise<geometry_msgs::PolygonStamped>
            ("offboard/cerca", 10)
    ros::Publisher cerca_max_pub = nh.advertise<geometry_msgs::PolygonStamped>
            ("offboard/cerca_extra", 10)
    ros::Publisher mensaje_camara_pub = nh.advertise<mavros_msgs::WaypointReached>
            ("offboard/mission/reached", 10)

    ros::Publisher kill_ros_pub = nh.advertise<std_msgs::Bool>
            ("kill_ROS", 10)

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb)
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_pose_cb)
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10)
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming")
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode")
    
    ROS_INFO("Parametros de vuelo:\n-Configuracion de camara:\n  Ancho del sensor        = %.3f mm\n  Altura del sensor       = %.3f mm\n  Pixeles por ancho       = %.0f pix\n  Pixeles por alto        = %.0f pix\n  Longitud focal          = %.2f mm\n-Configuracion de vuelo:\n  Altura de vuelo         = %.2f m\n  GSD                     = %.2f cm/pix\n  Translape lateral       = %.2f %%\n  Translape longitudinal  = %.2f %%\n  Angulo entrada          = %.2f \n",
            Width_sensor,Height_sensor,Image_pix_Width,Image_pix_Height,
            Focal_length,H,GSD,translape_lateral,translape_longitudinal,
            angulo_entrada)
###/cercas de vuelo 
    geometry_msgs::PolygonStamped cerca
    geometry_msgs::PolygonStamped cerca_max

    cerca.header.stamp=ros::Time::now()
    cerca.header.frame_id="map"

    geometry_msgs::Point32 p1

    p1.x=max_x
    p1.y=max_y
    p1.z=H
    cerca.polygon.points.push_back(p1)
    p1.x=max_x
    p1.y=min_y
    p1.z=H
    cerca.polygon.points.push_back(p1)
    p1.x=min_x
    p1.y=min_y
    p1.z=H
    cerca.polygon.points.push_back(p1)
    p1.x=min_x
    p1.y=max_y
    p1.z=H
    cerca.polygon.points.push_back(p1)

    cerca_max.header.stamp=ros::Time::now()
    cerca_max.header.frame_id="map"

    p1.x=max_cerca_x
    p1.y=max_cerca_y
    p1.z=H
    cerca_max.polygon.points.push_back(p1)
    p1.x=max_cerca_x
    p1.y=min_cerca_y
    p1.z=H
    cerca_max.polygon.points.push_back(p1)
    p1.x=min_cerca_x
    p1.y=min_cerca_y
    p1.z=H
    cerca_max.polygon.points.push_back(p1)
    p1.x=min_cerca_x
    p1.y=max_cerca_y
    p1.z=H
    cerca_max.polygon.points.push_back(p1)

    geometry_msgs::Vector3 vector_linea_y_max
    geometry_msgs::Point punto_linea_y_max
    vector_linea_y_max.x=1
    punto_linea_y_max.y=max_cerca_y
    punto_linea_y_max.z=H

    geometry_msgs::Vector3 vector_linea_y_min
    geometry_msgs::Point punto_linea_y_min
    vector_linea_y_min.x=1
    punto_linea_y_min.y=min_cerca_y
    punto_linea_y_min.z=H

    geometry_msgs::Vector3 vector_linea_x_max
    geometry_msgs::Point punto_linea_x_max
    vector_linea_x_max.y=1
    punto_linea_x_max.x=max_cerca_x
    punto_linea_x_max.z=H

    geometry_msgs::Vector3 vector_linea_x_min
    geometry_msgs::Point punto_linea_x_min
    vector_linea_x_min.y=1
    punto_linea_x_min.x=min_cerca_x
    punto_linea_x_min.z=H
### calculo de lineas de vuelo planas

    angulo_entrada=angulo_en_rango(angulo_entrada)
    geometry_msgs::Vector3 vector_avance=Normalizar_vector(cos(angulo_entrada*G_to_R),sin(angulo_entrada*G_to_R),0)
    vector_avance.x=vector_avance.x*base_en_aire
    vector_avance.y=vector_avance.y*base_en_aire
    geometry_msgs::Point punto_arranque,dis_entre_lineas
    dis_entre_lineas.x=separacion_lineas_vuelo*sin(angulo_entrada*G_to_R)
    dis_entre_lineas.y=separacion_lineas_vuelo*cos(angulo_entrada*G_to_R)
    #punto de arranque
    if(angulo_entrada<180 && angulo_entrada>90):
        ROS_INFO("ang1")
        dis_entre_lineas.x*=-1
        dis_entre_lineas.y*=-1
        punto_arranque.x= max_cerca_x
        punto_arranque.y= max_cerca_y
        punto_arranque.z= H
        punto_arranque.x= punto_arranque.x+dis_entre_lineas.x
        punto_arranque.y= punto_arranque.y-dis_entre_lineas.y
        punto_arranque=y_en_recta(vector_avance,punto_arranque,max_cerca_x)
        if(!Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
            punto_arranque=x_en_recta(vector_avance,punto_arranque,min_cerca_y)
        
    elif(angulo_entrada<90 && angulo_entrada>0):
        ROS_INFO("ang2")
        punto_arranque.x= min_cerca_x
        punto_arranque.y= max_cerca_y
        punto_arranque.z= H
        punto_arranque.x= punto_arranque.x+dis_entre_lineas.x
        punto_arranque.y= punto_arranque.y-dis_entre_lineas.y
        punto_arranque=y_en_recta(vector_avance,punto_arranque,min_cerca_x)
        if(!Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
            punto_arranque=x_en_recta(vector_avance,punto_arranque,min_cerca_y)
        
    elif(angulo_entrada<0 && angulo_entrada>-90):
        ROS_INFO("ang3")
        dis_entre_lineas.x*=-1
        dis_entre_lineas.y*=-1
        punto_arranque.x= min_cerca_x
        punto_arranque.y= min_cerca_y
        punto_arranque.z= H
        punto_arranque.x= punto_arranque.x+dis_entre_lineas.x
        punto_arranque.y= punto_arranque.y-dis_entre_lineas.y
        punto_arranque=y_en_recta(vector_avance,punto_arranque,min_cerca_x)
        if(!Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
            punto_arranque=x_en_recta(vector_avance,punto_arranque,max_cerca_y)
        
    elif(angulo_entrada<-90 && angulo_entrada>-180):
        ROS_INFO("ang4")
        punto_arranque.x= max_cerca_x
        punto_arranque.y= min_cerca_y
        punto_arranque.z= H
        punto_arranque.x= punto_arranque.x+dis_entre_lineas.x
        punto_arranque.y= punto_arranque.y-dis_entre_lineas.y
        punto_arranque=y_en_recta(vector_avance,punto_arranque,max_cerca_x)
        if(!Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
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
        if(!Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,px)):
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
            if( d1<=d2 && d1<=d3 && d1<=d4):
                punto_arranque=y_en_recta(vector_avance,punto_arranque,max_cerca_x)
                if(!Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                    if(d3<d4):
                        punto_arranque=x_en_recta(vector_avance,punto_arranque,max_cerca_y)
                    else:
                        punto_arranque=x_en_recta(vector_avance,punto_arranque,min_cerca_y)
                    
                
            elif( d2<=d3 && d2<=d4 ):
                punto_arranque=y_en_recta(vector_avance,punto_arranque,min_cerca_x)
                if(!Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                    if(d3<d4):
                        punto_arranque=x_en_recta(vector_avance,punto_arranque,max_cerca_y)
                    else:
                        punto_arranque=x_en_recta(vector_avance,punto_arranque,min_cerca_y)
                    
                
            elif( d3<=d4 ):
                punto_arranque=x_en_recta(vector_avance,punto_arranque,max_cerca_y)
                if(!Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                    if(d1<d2):
                        punto_arranque=y_en_recta(vector_avance,punto_arranque,max_cerca_x)
                    else:
                        punto_arranque=y_en_recta(vector_avance,punto_arranque,min_cerca_x)
                    
                
            else:
                punto_arranque=x_en_recta(vector_avance,punto_arranque,min_cerca_y)
                if(!Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                    if(d1<d2):
                        punto_arranque=y_en_recta(vector_avance,punto_arranque,max_cerca_x)
                    else:
                        punto_arranque=y_en_recta(vector_avance,punto_arranque,min_cerca_x)
                    
                
            
            i=0
            px=Recta(vector_avance,punto_arranque,i)
            if(!Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)):
                i=1000
                px.x=0
                px.y=0
            
        
        #ROS_INFO("punto en recta x=[%f],y=[%f],z=[%f]",px.x, px.y, px.z)
        pose.pose.position.x = px.x
        pose.pose.position.y = px.y
        pose.pose.position.z = px.z
        pose.pose.orientation=tf::createQuaternionMsgFromYaw(atan2(vector_avance.y,vector_avance.x))
        path.poses.push_back(pose)
    
    for (int i = 0 i < 5 ++i) :
        nav_pos_pub.publish(path)
        cerca_pub.publish(cerca)
        cerca_max_pub.publish(cerca_max)
        ros::spinOnce()
        rate.sleep()
    
    # wait for FCU connection
    while(ros::ok() && !current_state.connected):
        ros::spinOnce()
        rate.sleep()
    

    mavros_msgs::SetMode offb_set_mode
    offb_set_mode.request.custom_mode = "OFFBOARD"

    mavros_msgs::CommandBool arm_cmd
    arm_cmd.request.value = True

    ros::Time last_request = ros::Time::now()
    ros::Time last_view_porcentaje = ros::Time::now()
    bool mission=True
    int i=0
    while(ros::ok() && mission):
        geometry_msgs::PoseStamped pose=path.poses[i]
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))):
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent):
                ROS_INFO("Offboard enabled")
            
            last_request = ros::Time::now()
         else :
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))):
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success):
                    ROS_INFO("Vehicle armed")
                
                last_request = ros::Time::now()
            
        
        if(distancia(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,
                     current_local_pose.pose.position.x,current_local_pose.pose.position.y,current_local_pose.pose.position.z)<1):
            i++
            mavros_msgs::WaypointReached tomar_foto
            tomar_foto.header.frame_id = "map"
            tomar_foto.header.stamp = ros::Time::now()
            tomar_foto.wp_seq=i
            mensaje_camara_pub.publish(tomar_foto)
            if(i>path.poses.size()):
                mission=False
                std_msgs::Bool kill_ros
                kill_ros.data=True
                kill_ros_pub.publish(kill_ros)
            
        
        if(ros::Time::now() - last_view_porcentaje > ros::Duration(5.0)):
            ROS_INFO("Porcentade de mision %f%%",(i*100.0/(uint)path.poses.size()))
            last_view_porcentaje = ros::Time::now()
        
#        ROS_INFO("local point x=[%f],y=[%f],z=[%f]",current_local_pose.pose.position.x,
#            current_local_pose.pose.position.y,current_local_pose.pose.position.z)

        local_pos_pub.publish(pose)
        ros::spinOnce()
        rate.sleep()
    
