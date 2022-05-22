#include <ros/ros.h>
#include <mavros_msgs/WaypointReached.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
class Cap_imag{

public: Cap_imag(const std::string &_name);

private: void Image_Calback(const sensor_msgs::Image::ConstPtr &image);
private: void Trigerr_Calback(const std_msgs::Bool::ConstPtr &tri);

private: ros::NodeHandle nodo;

private: ros::Subscriber Image_subscriptor;
private: ros::Subscriber Trigerr_subscriptor;

private: int last_id;
private: bool capture_image;
};

Cap_imag::Cap_imag(const std::string &_name){
    ROS_INFO("init node");
//init subscriptor
    this->Image_subscriptor=this->nodo.subscribe<sensor_msgs::Image>("/iris_gimbal/usb_cam/image_raw",5,&Cap_imag::Image_Calback,this);
    this->Trigerr_subscriptor=this->nodo.subscribe<std_msgs::Bool>("/trigerr",2,&Cap_imag::Trigerr_Calback,this);

// init envioremnt variable
    this->last_id=0;
    this->capture_image=0;
}
void Cap_imag::Image_Calback(const sensor_msgs::Image::ConstPtr &image){
    if(this->capture_image){
        this->capture_image=0;
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(image,sensor_msgs::image_encodings::BGR8);
        std::string name="/home/andres1407/Capture_hillside/Captura_No_"+std::to_string(this->last_id)+".png";

        cv:: imwrite(name, cv_ptr->image);
        ROS_INFO("captura de imagen %d",this->last_id);
        this->last_id++;
    }
}
void Cap_imag::Trigerr_Calback(const std_msgs::Bool::ConstPtr &tri){
    this->capture_image=1;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "cap_imag_node");
    Cap_imag cap_imag("node");
    ros::spin();
    return 0;
}
