
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <string.h>
#include <math.h>

#define G_to_R 3.14159265/180

// configuracion de la camara
double Width_sensor=6.17;                // en mm
double Height_sensor=4.63;               // en mm
double Image_pix_Width=4608;             // en pixeles
double Image_pix_Height=3456;            // en pixeles
double Focal_length=4.9;                 // en mm
// configuracion de parametros de vuelo
double H=50;                             // en m
double translape_lateral=70;             // en %
double translape_longitudinal=70;        // en %
double angulo_entrada=70;                // en grados
double distancia_respuesta=20;           // en m
double gradiente_x=0;                    // en m
double gradiente_y=0;                    // en m
double gradiente_z=0;                    // en m
// cerca de vuelo
double min_x=-300;                       // en m desde el punto de partida
double min_y=-300;                       // en m desde el punto de partida
double max_x=300;                        // en m desde el punto de partida
double max_y=300;                        // en m desde el punto de partida

double distancia(double l_x,double l_y,double l_z,double s_x,double s_y,double s_z ){
    double dx=(l_x-s_x)*(l_x-s_x);
    double dy=(l_y-s_y)*(l_y-s_y);
    double dz=(l_z-s_z)*(l_z-s_z);
    return sqrt(dx+dy+dz);
}
bool Dentro_de_Cerca(double x_max, double x_min, double y_max, double y_min,geometry_msgs::Point p){
    if(p.x<=x_max && p.x>=x_min){
        if(p.y<=y_max && p.y>=y_min){
            return true;
        }else{
            return false;
        }
    }else{
        return false;
    }
}
double angulo_en_rango(double a){
    double aa;
    if (a>180){
        aa= a-360;
    }else if (a<-180){
        aa= a+360;
    }else{
        aa= a;
    }
    if(aa==180){
        aa=179;
    }else if(aa==90){
        aa=89;
    }else if(aa==0){
        aa=1;
    }else if(aa==-90){
        aa=-89;
    }else if(aa==-180){
        aa=-179;
    }
    return aa;
}
geometry_msgs::Vector3 Normalizar_vector(double x,double y, double z){
    double d=sqrt(x*x+y*y+z*z);
    geometry_msgs::Vector3 v;
    v.x=x/d;
    v.y=y/d;
    v.z=z/d;
    return v;
}

geometry_msgs::Point Recta(geometry_msgs::Vector3 v,geometry_msgs::Point p,int i ){
    geometry_msgs::Point Punto_Recta;
    Punto_Recta.x=v.x*i+p.x;
    Punto_Recta.y=v.y*i+p.y;
    Punto_Recta.z=p.z;
    return Punto_Recta;
}
geometry_msgs::Point x_en_recta(geometry_msgs::Vector3 v,geometry_msgs::Point p,double y ){
    geometry_msgs::Point Punto_Recta;
    Punto_Recta.x=(y-p.y)*v.x/v.y+p.x;
    Punto_Recta.y=y;
    Punto_Recta.z=p.z;
    return Punto_Recta;
}
geometry_msgs::Point y_en_recta(geometry_msgs::Vector3 v,geometry_msgs::Point p,double x ){
    geometry_msgs::Point Punto_Recta;
    Punto_Recta.x=x;
    Punto_Recta.y=(x-p.x)*v.y/v.x+p.y;
    Punto_Recta.z=p.z;
    return Punto_Recta;
}


geometry_msgs::Point Plano_Z(geometry_msgs::Vector3 v,geometry_msgs::Point p_plano,geometry_msgs::Point p_x_y ){
    geometry_msgs::Point Punto_Plano;
    Punto_Plano.x=p_x_y.x;
    Punto_Plano.y=p_x_y.y;
    Punto_Plano.z=p_plano.z-(v.x*(p_x_y.x-p_plano.x)+v.y*(p_x_y.y-p_plano.y))/v.z;
    return Punto_Plano;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped  current_local_pose;
void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_local_pose = *msg;
}

int main(int argc, char **argv)
{
    //double H = atof(argv[1]);
    //calculo de variables de vuelo
    double GSD=(Width_sensor*H*100)/(Focal_length*Image_pix_Width);             // en cm/pix
    double ancho_huella=(GSD*Image_pix_Width)/100;                              // en m
    double alto_huella=(GSD*Image_pix_Height)/100;                              // en m
    double separacion_lineas_vuelo=ancho_huella*(1-(translape_lateral/100));    // en m
    double base_en_aire=alto_huella*(1-(translape_longitudinal/100));           // en m

    double min_cerca_x=min_x-distancia_respuesta;
    double min_cerca_y=min_y-distancia_respuesta;
    double max_cerca_x=max_x+distancia_respuesta;
    double max_cerca_y=max_y+distancia_respuesta;


    ros::init(argc, argv, "offb_node_rute_square");
    ros::NodeHandle nh;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);


    ros::Publisher nav_pos_pub = nh.advertise<nav_msgs::Path>
            ("offboard/nav", 10);
    ros::Publisher cerca_pub = nh.advertise<geometry_msgs::PolygonStamped>
            ("offboard/cerca", 10);
    ros::Publisher cerca_max_pub = nh.advertise<geometry_msgs::PolygonStamped>
            ("offboard/cerca_extra", 10);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, local_pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    
    ROS_INFO("Parametros de vuelo:\n-Configuracion de camara:\n  Ancho del sensor        = %.3f mm\n  Altura del sensor       = %.3f mm\n  Pixeles por ancho       = %.0f pix\n  Pixeles por alto        = %.0f pix\n  Longitud focal          = %.2f mm\n-Configuracion de vuelo:\n  Altura de vuelo         = %.2f m\n  GSD                     = %.2f cm/pix\n  Translape lateral       = %.2f %%\n  Translape longitudinal  = %.2f %%\n  Angulo entrada          = %.2f \n",
            Width_sensor,Height_sensor,Image_pix_Width,Image_pix_Height,
            Focal_length,H,GSD,translape_lateral,translape_longitudinal,
            angulo_entrada);
///////cercas de vuelo 
    geometry_msgs::PolygonStamped cerca;
    geometry_msgs::PolygonStamped cerca_max;

    cerca.header.stamp=ros::Time::now();
    cerca.header.frame_id="map";

    geometry_msgs::Point32 p1;

    p1.x=max_x;
    p1.y=max_y;
    p1.z=H;
    cerca.polygon.points.push_back(p1);
    p1.x=max_x;
    p1.y=min_y;
    p1.z=H;
    cerca.polygon.points.push_back(p1);
    p1.x=min_x;
    p1.y=min_y;
    p1.z=H;
    cerca.polygon.points.push_back(p1);
    p1.x=min_x;
    p1.y=max_y;
    p1.z=H;
    cerca.polygon.points.push_back(p1);

    cerca_max.header.stamp=ros::Time::now();
    cerca_max.header.frame_id="map";

    p1.x=max_cerca_x;
    p1.y=max_cerca_y;
    p1.z=H;
    cerca_max.polygon.points.push_back(p1);
    p1.x=max_cerca_x;
    p1.y=min_cerca_y;
    p1.z=H;
    cerca_max.polygon.points.push_back(p1);
    p1.x=min_cerca_x;
    p1.y=min_cerca_y;
    p1.z=H;
    cerca_max.polygon.points.push_back(p1);
    p1.x=min_cerca_x;
    p1.y=max_cerca_y;
    p1.z=H;
    cerca_max.polygon.points.push_back(p1);
    
////// calculo de lineas de vuelo planas

    angulo_entrada=angulo_en_rango(angulo_entrada);
    geometry_msgs::Vector3 vector_avance=Normalizar_vector(cos(angulo_entrada*G_to_R),sin(angulo_entrada*G_to_R),0);
    vector_avance.x=vector_avance.x*base_en_aire;
    vector_avance.y=vector_avance.y*base_en_aire;
    geometry_msgs::Point punto_arranque,dis_entre_lineas;
    dis_entre_lineas.x=separacion_lineas_vuelo*sin(angulo_entrada*G_to_R);
    dis_entre_lineas.y=separacion_lineas_vuelo*cos(angulo_entrada*G_to_R);
    ROS_INFO("distancia de rectas x=[%f],y=[%f],z=[%f]",dis_entre_lineas.x, dis_entre_lineas.y, dis_entre_lineas.z);
    //punto de arranque
    if(angulo_entrada<180 && angulo_entrada>90){
        punto_arranque.x=max_cerca_x;
        punto_arranque.y= max_cerca_y;
    }else if(angulo_entrada<90 && angulo_entrada>0){
        punto_arranque.x= min_cerca_x;
        punto_arranque.y= max_cerca_y;;
        punto_arranque.z=H;
        for (int j = 0; j < 100; ++j){
            punto_arranque.x= punto_arranque.x+dis_entre_lineas.x;
            punto_arranque.y= punto_arranque.y-dis_entre_lineas.y;
            punto_arranque=x_en_recta(vector_avance,punto_arranque,max_cerca_y);
            ROS_INFO("punto arranque en recta x x=[%f],y=[%f],z=[%f]",punto_arranque.x, punto_arranque.y, punto_arranque.z);
            if(!Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,punto_arranque)){
                punto_arranque=y_en_recta(vector_avance,punto_arranque,max_cerca_x);
                ROS_INFO("punto arranque en recta y x=[%f],y=[%f],z=[%f]",punto_arranque.x, punto_arranque.y, punto_arranque.z);
            }
            for (int i = 0; i < 100; ++i){
                geometry_msgs::Point px=Recta(vector_avance,punto_arranque,i);
                ROS_INFO("punto en recta x=[%f],y=[%f],z=[%f]",px.x, px.y, px.z);
                if(Dentro_de_Cerca(max_x,min_x,max_y,min_y,px)){
                    j=100;
                    ROS_INFO("dentro de cerca");
                    break;
                }
                if(!Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,px)){
                    ROS_INFO("cambio de linea");
                    break;
                }
            }
        }
    }else if(angulo_entrada<0 && angulo_entrada>-90){
        punto_arranque.x=max_cerca_x;
        punto_arranque.y= max_cerca_y;
    }else if(angulo_entrada<-90 && angulo_entrada>-180){
        punto_arranque.x=min_cerca_x;
        punto_arranque.y= max_cerca_y;
    }

    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id= "map";

    ROS_INFO("path");
    for (int i = 0; i < 100; ++i){
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map";

        geometry_msgs::Point px=Recta(vector_avance,punto_arranque,i);
        if(!Dentro_de_Cerca(max_cerca_x,min_cerca_x,max_cerca_y,min_cerca_y,px)){
            break;
        }
        ROS_INFO("punto en recta x=[%f],y=[%f],z=[%f]",px.x, px.y, px.z);
        pose.pose.position.x = px.x;
        pose.pose.position.y = px.y;
        pose.pose.position.z = px.z;
        pose.pose.orientation=tf::createQuaternionMsgFromYaw(atan2(vector_avance.y,vector_avance.x));
        path.poses.push_back(pose);
    }
    while(ros::ok()){
        nav_pos_pub.publish(path);
        cerca_pub.publish(cerca);
        cerca_max_pub.publish(cerca_max);
        ros::spinOnce();
        rate.sleep();
    }

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = H;
    // pose.pose.orientation=tf::createQuaternionMsgFromYaw(atan2(1,0));

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

            // pose.pose.position.x = 0;
            // pose.pose.position.y = 0;
            // pose.pose.position.z = H;
            // pose.pose.orientation=tf::createQuaternionMsgFromYaw(atan2(1,0));
        // ROS_INFO("set point x=[%f],y=[%f],z=[%f]",pose.pose.position.x,pose.pose.position.y,
        //     pose.pose.position.z);
        ROS_INFO("local point x=[%f],y=[%f],z=[%f]",current_local_pose.pose.position.x,
            current_local_pose.pose.position.y,current_local_pose.pose.position.z);
    
        // local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
