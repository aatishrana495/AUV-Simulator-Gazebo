#ifndef _AUV_PLUGIN_HH_
#define _AUV_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <functional>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include<string>
#include<iostream>
#include<sim_gazebo/ThrusterSpeeds.h>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class AuvPlugin : public ModelPlugin
  {
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;
    protected: physics::LinkPtr hull;
    private: physics::LinkPtr thrusters[6];
    private: std::string path="auv::auv::thruster_";
    private: std::string k;
  protected: double initial,finalForce,adjusted,hullPose;
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      //output message:
      std::cerr << "\nThe Auv plugin is attach to model " <<_model->GetName() << "\n";

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",ros::init_options::NoSigintHandler);
      }

      this->model = _model;
      this->hull = _model->GetChildLink("auv::auv::hull");
      this->initial=0.0;
      this->finalForce=0.0;
      this->adjusted=0.0;
      this->hullPose=0.0;
      for(int i=0;i<6;i++){
        k=std::to_string(i+1);
        this->thrusters[i]=_model->GetChildLink(path+k);
      }

      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =ros::SubscribeOptions::create<sim_gazebo::ThrusterSpeeds>("/thruster_speeds",1,boost::bind(&AuvPlugin::OnRosMsg, this, _1),ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =std::thread(std::bind(&AuvPlugin::QueueThread, this));
    }

    public: void OnRosMsg(const sim_gazebo::ThrusterSpeeds::ConstPtr &_msg)
    {
      hullPose=this->hull->GetWorldCoGPose().pos[2];
      if(hullPose<0.0){
      std::cerr << "\nThe force value is [" <<_msg->data[0]<< "]\n";
      this->thrusters[0]->AddRelativeForce(ignition::math::Vector3d(0, 0, -AdjustForce(_msg->data[0])));
      this->thrusters[1]->AddRelativeForce(ignition::math::Vector3d(0, 0, -AdjustForce(_msg->data[1])));
      this->thrusters[2]->AddRelativeForce(ignition::math::Vector3d(0, 0, AdjustForce(-_msg->data[2])));
      this->thrusters[3]->AddRelativeForce(ignition::math::Vector3d(-AdjustForce(_msg->data[3]), 0, 0));
      this->thrusters[4]->AddRelativeForce(ignition::math::Vector3d(-AdjustForce(_msg->data[4]), 0, 0));
      this->thrusters[5]->AddRelativeForce(ignition::math::Vector3d(0, -AdjustForce(_msg->data[5]), 0));
    }
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

  public: double AdjustForce(double initial)
    {
      if (initial >= 1470 && initial <= 1530) {
        adjusted = 0.0f;
      //			Debug.Log ("Adjusted = " + adjusted + " for thruster " + thrusterNumber);
      }
      else if (initial > 1530) {
        initial = initial- 1530;
        adjusted =(initial / 370.0f) * 2.36f;
      } else {
        initial -= 1470;
        adjusted = initial / 370.0f * 1.85f;
      }
      adjusted *= 9.8f * 1000.0f;
      //		Debug.Log ("Adjusted before return = " + adjusted + " for thruster " + thrusterNumber);
      return adjusted;
    }

  public: double AddForce(double f){
    if(hullPose<0.0f){
    finalForce=AdjustForce(f);
  }else{
    finalForce=0.0;
  }
  return finalForce;
  }

  };
  GZ_REGISTER_MODEL_PLUGIN(AuvPlugin)
}
#endif
