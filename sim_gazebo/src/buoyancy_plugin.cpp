#ifndef _BUOYANCY_ORIGINAL_H
#define _BUOYANCY_ORIGINAL_H

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>

namespace gazebo{
  class BuoyancyOriginal : public ModelPlugin{
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to model containing the plugin.
    protected: physics::ModelPtr auv;
    protected: physics::LinkPtr hull;

    /// \brief Pointer to the physics engine (for accessing gravity).
    protected: physics::PhysicsEnginePtr physicsEngine;

    /// \brief Pointer to the plugin SDF.
    protected: sdf::ElementPtr sdf;
    protected: float waterLevel, floatHeight,bounceDamp,forcefactor,vel;
    protected: math::Vector3 actionPoint,buoyancyCentreOffset;

    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {

      GZ_ASSERT(_model != NULL, "Received NULL model pointer");
      this->auv = _model;
      physics::WorldPtr world = _model->GetWorld();
      GZ_ASSERT(world != NULL, "Model is in a NULL world");
      this->physicsEngine = world->GetPhysicsEngine();
      GZ_ASSERT(this->physicsEngine != NULL, "Physics engine was NULL");
      GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer");
      this->sdf = _sdf;

      this->hull = _model->GetChildLink("auv::auv::hull");

      updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&BuoyancyOriginal::OnUpdate, this));

      this->waterLevel=0.0;
      this->floatHeight=1.8;
      this->bounceDamp=0.0;
      std::cerr << "\nThe Buoyancy plugin is attach to model " <<_model->GetName() << "\n";
    }

    public: void OnUpdate() {
      actionPoint=this->hull->GetWorldCoGPose().pos;
      //std::cout<<actionPoint<<std::endl;
      forcefactor=1.0-((actionPoint.z-waterLevel)/floatHeight);
      vel=fabs(this->hull->GetRelativeLinearVel().z);
      bounceDamp=vel*2.5;
      //std::cout<<actionPoint.z<<std::endl;
      if (forcefactor>0.0){
        this->hull->AddForce(math::Vector3(0,0,13*9.8*(forcefactor-(vel*bounceDamp))));
        //std::cout<<13*9.8*forcefactor-(vel*bounceDamp)<<std::endl;

      }
    }

  };
  GZ_REGISTER_MODEL_PLUGIN(BuoyancyOriginal)
}
#endif
