#ifndef _GAZEBO_MINHO_HARDWARE_HH_
#define _GAZEBO_MINHO_HARDWARE_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Events.hh>
#include "ros/ros.h"

#define DEG_TO_RAD M_PI/180.0
#define RAD_TO_DEG 180.0/M_PI
#define REVOLUTE_JOINT 576

#define maxSteer 30*DEG_TO_RAD
#define maxVel 60.0f
#include <iostream>
#include <map>

namespace gazebo
{
  /// \brief A plugin to control a soccer robot's omnidirectional movement, kicker and dribbler
  class CACart : public ModelPlugin
  {
    public: 
    
    /// \brief Constructor. Initialized deafult variables for various variables
    CACart();

    /// \brief Destructor
    virtual ~CACart();
    
    /// \brief Plugin Load function. Initializes all ros topics for the robot model,
    /// also starting message queue thread. Connects gazebo events like world update,
    /// time reset and world reset
    /// \param _parent - Model pointer to the model defining this plugin
    /// \param _sdf - pointer to the SDF of the model
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    
    void onUpdate();
    
    void updateVelocity();
    
    void updateDirection();
    
    void onControlData(ConstVector2dPtr &_msg);
    private:
    // VARIABLES
        
    /// \brief Pointer to the model that defines this plugin
    physics::ModelPtr m_model;
    /// \brief Transport node used to communicate with the transport system
    transport::NodePtr m_node;

    /// \brief pointer to server update event
    event::ConnectionPtr m_updateConn, m_updateConnEnd;
    
    physics::JointControllerPtr m_ctrl;
    
    physics::JointPtr m_rrJoint, m_rlJoint;
    physics::LinkPtr m_rWheel, m_lWheel;
    ignition::math::Pose3d m_rWheelPose, m_lWheelPose;
    float m_baseRWheelYaw, m_baseLWheelYaw;
    
    common::PID velController, dirController;
    
    bool jointsInitialized;
    
    common::Time m_prevUpdateTime;

    transport::SubscriberPtr m_controlSub;
    
    float targetVelocity, targetDirection, last_tv, last_td;
  };
}
#endif
