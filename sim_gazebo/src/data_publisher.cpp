#ifndef _DATA_PUBLISHER_HH_
#define _DATA_PUBLISHER_HH_

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
#include<sim_gazebo/Combined.h>
#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <boost/scoped_ptr.hpp>
#include <boost/algorithm/string.hpp>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class DataPublisher : public ModelPlugin
  {
    //private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Publisher rosPub;
    private: sim_gazebo::Combined msg;
    /// \brief Pointer to the model.
    private: physics::ModelPtr auv;
    private: physics::LinkPtr hull;
    private: gazebo::event::ConnectionPtr updateConnection;
    private: gazebo::physics::WorldPtr sauvc;
    private: boost::shared_ptr<ros::NodeHandle> node;
    private: double updateRate;
    private: double updatePeriod;
    private: gazebo::common::Time lastUpdate;

    public: void Load(physics::ModelPtr _auv, sdf::ElementPtr _sdf)
    {
      //std::this_thread::sleep_for(std::chrono::milliseconds(1000*100));
      this->auv = _auv;
      //output message:
      GZ_ASSERT(this->auv != NULL, "Invalid model pointer");
      std::cerr << "\nThe data_publisher plugin is attach to model[" <<_auv->GetName() << "]\n";
      this->sauvc = this->auv->GetWorld();
      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        gzerr << "ROS was not initialized. Closing plugin..." << std::endl;
        return;
      }
      this->node = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("gazebo_publish"));
      this->hull=_auv->GetChildLink("auv::auv::hull");

      if (_sdf->HasElement("updateRate"))
        this->updateRate = _sdf->Get<double>("updateRate");
      else
        this->updateRate = 50;


      GZ_ASSERT(this->updateRate > 0, "Update rate must be positive");
      // Setting the update period
      this->updatePeriod = 1.0 / this->updateRate;
      this->rosPub =this->node->advertise<sim_gazebo::Combined>("/synchronizer/combined", 1);

    #if GAZEBO_MAJOR_VERSION >= 8
        this->lastUpdate = this->sauvc->SimTime();
      #else
        this->lastUpdate = this->sauvc->GetSimTime();
      #endif

      // Connect the update function to the Gazebo callback
      this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&DataPublisher::Onupdate, this, _1));
        //std::cerr<<this->hull->WorldPose().Rot().X();
      //this->rosNode.reset(new ros::NodeHandle("gazebo_publish"));
      //this->rosPub=this->rosNode->advertise<auvsim_gazebo::Combined>("/synchronizer/combined",1);
      //update_event_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&DataPulisher::OnUpdate, this));
      //std::this_thread::sleep_for(std::chrono::milliseconds(10000));
      //this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DataPulisher::onUpdate, this));
    }

    public: void Onupdate(const gazebo::common::UpdateInfo &_info){
      #if GAZEBO_MAJOR_VERSION >= 8
        gazebo::common::Time simTime = this->sauvc->SimTime();
      #else
        gazebo::common::Time simTime = this->sauvc->GetSimTime();
      #endif

      if (simTime - this->lastUpdate >= this->updatePeriod)
      {
        this->msg.angular[0]=this->hull->GetWorldPose().rot.x;
        this->msg.angular[1]=this->hull->GetWorldPose().rot.y;
        this->msg.angular[2]=this->hull->GetWorldPose().rot.z;
        this->msg.linear[0]=this->hull->GetWorldPose().pos.x;
        this->msg.linear[1]=this->hull->GetWorldPose().pos.y;
        this->msg.linear[2]=this->hull->GetWorldPose().pos.z;
        this->msg.depth=-(this->hull->GetWorldPose().pos.z);
        this->rosPub.publish(msg);
        this->lastUpdate = simTime;
    }
      //buoyancy:

    }
  };
  GZ_REGISTER_MODEL_PLUGIN(DataPublisher)
}
#endif
