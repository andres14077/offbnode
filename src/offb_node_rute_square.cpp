
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <string.h>
#include <math.h>

// configuracion de la camara
double Width_sensor=6.17;               // en mm
double Height_sensor=4.63;              // en mm
double Image_pix_Width=4608;
double Image_pix_Height=3456;
double Focal_length=4.9;                // en mm
// configuracion de parametros de vuelo
double H=50;                            // en m
double translape_lateral=70;            // en %
double translape_longitudinal=70;       // en %
double angulo_entrada=70;               // en grados
double distancia_respuesta=20;               // en m
double gradiente_x=0;                    // en m
double gradiente_y=0;                    // en m
double gradiente_z=0;                    // en m
// cerca de vuelo
double min_x=-300;                    // en m desde el punto de partida
double min_y=-300;                    // en m desde el punto de partida
double max_x=300;                    // en m desde el punto de partida
double max_y=300;                    // en m desde el punto de partida

double distancia(double l_x,double l_y,double l_z,double s_x,double s_y,double s_z ){
    double dx=(l_x-s_x)*(l_x-s_x);
    double dy=(l_y-s_y)*(l_y-s_y);
    double dz=(l_z-s_z)*(l_z-s_z);
    return sqrt(dx+dy+dz);
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



    ros::init(argc, argv, "offb_node_rute_square");
    ros::NodeHandle nh;

    ros::Publisher nav_pos_pub = nh.advertise<nav_msgs::Path>
            ("iris/nav", 10);
    nav_msgs::Path path;
    for (int i = 0; i < 100; ++i)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = i;
        pose.header.frame_id = "map";
        pose.pose.position.x = 10*cos(62.8318e-3*i);
        pose.pose.position.y = 10*sin(62.8318e-3*i);
        pose.pose.position.z = 10*cos(62.8318e-3*i);
        pose.pose.orientation=tf::createQuaternionMsgFromYaw(atan2(pose.pose.position.y,pose.pose.position.x));
        path.poses.push_back(pose);
    }
    nav_pos_pub.publish(path);

    return 0;
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

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    //geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = H;
    pose.pose.orientation=tf::createQuaternionMsgFromYaw(atan2(1,0));

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
        ROS_INFO("set point x=[%f],y=[%f],z=[%f]",pose.pose.position.x,pose.pose.position.y,
            pose.pose.position.z);
        ROS_INFO("local point x=[%f],y=[%f],z=[%f]",current_local_pose.pose.position.x,
            current_local_pose.pose.position.y,current_local_pose.pose.position.z);
    
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
