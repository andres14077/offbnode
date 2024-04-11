#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
class DistanceToGroundPlugin : public ModelPlugin
{
    private: physics::ModelPtr robot;
    private: physics::WorldPtr world;
    private: ros::NodeHandle nh;
    private: ros::Publisher distance_pub;
    private: event::ConnectionPtr updateConnection;

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        // Inicializa ROS y el publicador
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "distance_to_ground_node");
        nh = ros::NodeHandle();


        this->robot = _parent;
        std::string distanceTopic = this->robot->GetName() + "/distance_to_ground";
        distance_pub = nh.advertise<std_msgs::Float64>(distanceTopic, 10);

        // // Asegurarse de que el parámetro "robot_name" esté definido
        // if (!_sdf->HasElement("robot_name"))
        // {
        //     gzerr << "No se especificó el nombre del robot para el plugin DistanceToGround.\n";
        //     return;
        // }
        // std::string robot_name = _sdf->Get<std::string>("robot_name");

        // Obtiene la referencia al modelo del robot usando el nombre
        // this->robot = _parent->GetWorld()->ModelByName(robot_name);


        // // Verifica que el modelo del robot se obtuvo correctamente
        // if (!this->robot)
        // {
        //     gzerr << "No se pudo encontrar un modelo con el nombre " << robot_name << ".\n";
        //     return;
        // }

        // Guarda una referencia al mundo
        this->world = _parent->GetWorld();

        // Llama a la función Update cada vez que se actualiza la simulación
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&DistanceToGroundPlugin::OnUpdate, this));
    }

    public: void OnUpdate()
    {
        // Define el punto de inicio y fin del raycast
        ignition::math::Vector3d start = this->robot->WorldPose().Pos()- ignition::math::Vector3d(0, 0, 0.18);
        ignition::math::Vector3d end = start - ignition::math::Vector3d(0, 0, 500);  // Asumimos que 100m es suficiente

        // Realiza el raycast
        physics::RayShapePtr ray = boost::dynamic_pointer_cast<physics::RayShape>(
            this->world->Physics()->CreateShape("ray", physics::CollisionPtr()));
        ray->SetPoints(start, end);
        double dist;
        std::string entity;
        ray->GetIntersection(dist, entity);
        // Publica la distancia en el tópico de ROS
        std_msgs::Float64 msg;
        msg.data = dist;
        distance_pub.publish(msg);
    }
};
GZ_REGISTER_MODEL_PLUGIN(DistanceToGroundPlugin)
}
