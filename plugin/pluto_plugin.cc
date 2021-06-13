#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/Imu.h"
#include "fly_bot_cpp/kwad_input.h"
#include "fly_bot_cpp/kwad_state.h"
#include <functional>
#include <iostream>
#include <map>

namespace gazebo{
	//plugin class. Extends model plugin
	class KwadControlPlugin : public ModelPlugin
    {
        public: KwadControlPlugin() {}

        public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            this->model = _model;
            this->body = _model->GetChildLink("body");
            std::cout << "number of joints in model: " << this->model->GetJointCount() << std::endl;
               
            if (_model->GetJointCount() == 0) {
                std::cerr << "model not loaded" << std::endl;
                return;
            }

            if (!ros::isInitialized()) {
                ROS_WARN("A ROS node for Gazebo has not been initialized, unable to load plugin.");
                ROS_WARN("Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
                ROS_WARN("If using gazebo as a stand-alone, package, run roscore first");
                ROS_WARN("Trying to initialize ROS node - 'gazebo_client'\n");

                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "gazebo_client",
                          ros::init_options::NoSigintHandler);
            }

            std::cout << "line 44" << std::endl;
            this->rosNode.reset(new ros::NodeHandle());
            std::cout << "line 46" << std::endl;
            // create a topic for control inputs and subscribe to it
            this->controlSub = this->rosNode->subscribe(
                this->model->GetName() + "/control_cmd",
                10,
                &KwadControlPlugin::controlCallback,
                this);
            
            std::cout << "line 54" << std::endl;
            // create a subscriber for IMU sensor
            this->imuSub = 
                this->rosNode->subscribe(
                    this->model->GetName() + "/imu",
                    10,
                    &KwadControlPlugin::imuCallback,
                    this);
            std::cout << "line 62" << std::endl;
            // create topic for 12 states of kwad and advertise
            this->statePub = this->rosNode->advertise<fly_bot_cpp::kwad_state>(
                this->model->GetName() + "/twelve_state", 10);

            std::cout << "line 67" << std::endl;
            /* namespace created by sdf?
            if (!sdf_->HasElement("robotNamespace")) {
                ROS_FATAL_STREAM("Missing parameter <namespace> in plugin");
                return;
            } else {
                pluginNamespace = _sdf->GetElement("robotNamespace")->GetValue->GetAsString();
            }
            */
            std::cout << "line 76" << std::endl;
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&KwadControlPlugin::OnUpdate, this));
            
            ROS_INFO("KwadControlPlugin loaded\n");
        }

        private: void OnUpdate()
        {
            if (this->controlSub.getNumPublishers() < 1) {
                this->controlMsg.thrust = 10;
                this->controlMsg.tau_x = 0;
                this->controlMsg.tau_y = 0;
                this->controlMsg.tau_z = 0;
            }
            if (this->imuSub.getNumPublishers() < 1) {
                /*
                this->imuData.orientation.x = 0;
                this->imuData.orientation.y = 0;
                this->imuData.orientation.z = 0;
                this->imuData.orientation.w = 0;
                this->imuData.angular_velocity.x = 0;
                this->imuData.angular_velocity.y = 0;
                this->imuData.angular_velocity.z = 0;
                this->imuData.linear_acceleration.x = 0;
                this->imuData.linear_acceleration.y = 0;
                this->imuData.linear_acceleration.z = 0;
                */
                this->q.Set(0, 0, 0, 0);
                this->ang_vel.Set(0, 0, 0);
            }

            // apply forces and torques to body link
            this->control_force.Set(0, 0, this->controlMsg.thrust);

            this->body->SetForce(this->control_force);
            
            this->control_torque.Set(this->controlMsg.tau_x, 
                               this->controlMsg.tau_y,
                               this->controlMsg.tau_z);
            this->body->SetTorque(this->control_torque);
            
            // make state vector and publish to Kwad/twelve_state topic
            this->pose = this->body->WorldPose();
            this->lin_vel = this->body->WorldLinearVel();

            this->stateMsg.x = this->pose.Pos().X();
            this->stateMsg.y = this->pose.Pos().Y();
            this->stateMsg.z = this->pose.Pos().Z();
            this->stateMsg.x_dot = this->lin_vel.X();
            this->stateMsg.y_dot = this->lin_vel.Y();
            this->stateMsg.z_dot = this->lin_vel.Z();
            this->stateMsg.phi = this->q.Roll();
            this->stateMsg.theta = this->q.Pitch();
            this->stateMsg.psi = this->q.Yaw();
            this->stateMsg.p = this->ang_vel[0];
            this->stateMsg.q = this->ang_vel[1];
            this->stateMsg.r = this->ang_vel[2];

            this->statePub.publish(this->stateMsg);

            return;
        }

        private: void controlCallback(const fly_bot_cpp::kwad_input::ConstPtr& msg)
        {
            this->controlMsg.thrust = msg->thrust;
            this->controlMsg.tau_x = msg->tau_x;
            this->controlMsg.tau_y = msg->tau_y;
            this->controlMsg.tau_z = msg->tau_z;
        }

        private: void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
        {
            /*
            this->imuData.orientation.x = msg->orientation.x;
            this->imuData.orientation.y = msg->orientation.y;
            this->imuData.orientation.z = msg->orientation.z;
            this->imuData.orientation.w = msg->orientation.w;
            this->imuData.angular_velocity.x = msg->angular_velocity.x;
            this->imuData.angular_velocity.y = msg->angular_velocity.y;
            this->imuData.angular_velocity.z = msg->angular_velocity.z;
            */

            this->q.Set(
                msg->orientation.x, msg->orientation.y,
                msg->orientation.z, msg->orientation.z);
            this->ang_vel.Set(
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z);
        }

        private: physics::ModelPtr model;
        private: physics::LinkPtr body;

        private: fly_bot_cpp::kwad_input controlMsg;
        private: fly_bot_cpp::kwad_state stateMsg;
        //private: sensor_msgs::Imu imuData;
        private: ignition::math::Pose3<double> pose;
        private: ignition::math::Vector3<double> lin_vel;
        private: ignition::math::Vector3<double> ang_vel;
        private: ignition::math::Quaternion<double> q;
        private: ignition::math::Vector3<double> control_force;
        private: ignition::math::Vector3<double> control_torque;

        private: std::unique_ptr<ros::NodeHandle> rosNode;
        private: ros::Subscriber controlSub;
        private: ros::Subscriber imuSub;
        private: ros::Publisher statePub;
        
        private: event::ConnectionPtr updateConnection;

    };

    GZ_REGISTER_MODEL_PLUGIN(KwadControlPlugin)
}
