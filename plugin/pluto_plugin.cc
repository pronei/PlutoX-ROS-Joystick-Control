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

            this->rosNode.reset(new ros::NodeHandle());
            
            // create a topic for control inputs and subscribe to it
            this->controlSub = this->rosNode->subscribe(
                this->model->GetName() + "/control_cmd",
                10,
                &KwadControlPlugin::controlCallback,
                this);
            
            // create a subscriber for IMU sensor
            this->imuSub = 
                this->rosNode->subscribe(
                    this->model->GetName() + "/imu",
                    10,
                    &KwadControlPlugin::imuCallback,
                    this);
            
            // create topic for 12 states of kwad and advertise
            this->statePub = this->rosNode->advertise<fly_bot_cpp::kwad_state>(
                this->model->GetName() + "/twelve_state", 10);

            /* namespace created by sdf?
            if (!sdf_->HasElement("robotNamespace")) {
                ROS_FATAL_STREAM("Missing parameter <namespace> in plugin");
                return;
            } else {
                pluginNamespace = _sdf->GetElement("robotNamespace")->GetValue->GetAsString();
            }
            */

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&KwadControlPlugin::OnUpdate, this));
            
            ROS_INFO("KwadControlPlugin loaded\n");
        }

        private: void OnUpdate()
        {
            if (this->controlSub.getNumPublishers() < 1) {
                this->controlMsg.thrust = 0;
                this->controlMsg.tau_x = 0;
                this->controlMsg.tau_y = 0;
                this->controlMsg.tau_z = 0;
            }
            if (this->imuSub.getNumPublishers() < 1) {
                this->q.Set(0, 0, 0, 0);
                this->ang_vel.Set(0, 0, 0);
            }

            // apply forces and torques to body link
            this->control_force.Set(0, 0, this->controlMsg.thrust);
            this->body->AddRelativeForce(this->control_force);
            

            this->control_torque.Set(this->controlMsg.tau_x, 
                                     this->controlMsg.tau_y,
                                     this->controlMsg.tau_z);
            this->body->AddRelativeTorque(this->control_torque);
            
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
            this->q.Set(
                msg->orientation.x, msg->orientation.y,
                msg->orientation.z, msg->orientation.z);
            this->ang_vel.Set(
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z);
        }

        private: double clamp(double const &x, double const &lb, double const &ub)
        {
            return std::max(std::min(x, ub), lb);
        }

        private:
            physics::ModelPtr model;
            physics::LinkPtr body;

            //const double max_thrust, max_tau_xy, max_tau_z;

            fly_bot_cpp::kwad_input controlMsg;
            fly_bot_cpp::kwad_state stateMsg;
            ignition::math::Pose3<double> pose;
            ignition::math::Vector3<double> lin_vel;
            ignition::math::Vector3<double> ang_vel;
            ignition::math::Quaternion<double> q;
            ignition::math::Vector3<double> control_force;
            ignition::math::Vector3<double> control_torque;

            std::unique_ptr<ros::NodeHandle> rosNode;
            ros::Subscriber controlSub;
            ros::Subscriber imuSub;
            ros::Publisher statePub;
            
            event::ConnectionPtr updateConnection;

    };

    GZ_REGISTER_MODEL_PLUGIN(KwadControlPlugin)
}
