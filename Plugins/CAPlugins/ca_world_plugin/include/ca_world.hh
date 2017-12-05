#ifndef _CA_WORLD_HH_
#define _CA_WORLD_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector2.hh>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <vector>
#include <sdf/Param.hh>
#include <sdf/sdf.hh>
#include "ros/ros.h"
#include <cstdlib>
#include <fstream>
#define DEG_TO_RAD M_PI/180.0
#define RAD_TO_DEG 180.0/M_PI

namespace gazebo
{
  /// \brief A plugin to control a soccer robot's omnidirectional movement, kicker and dribbler
  class CAWorld : public WorldPlugin
  {

    
    public: 
    /// \brief Constructor
    CAWorld();

    /// \brief Destructor
    virtual ~CAWorld();
    
    /// \brief Plugin Load function
    /// \param _parent - Model pointer to the model defining this plugin
    /// \param _sdf - pointer to the SDF of the model
    void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
    
    void onStateData(ConstIntPtr &_msg);
    
    void onWorldUpdate();
    private:
    // VARIABLES
    /// \brief Transport node used to communicate with the transport system
    transport::NodePtr m_node;
    /// \brief pointer to the world object of the simulation
    physics::WorldPtr m_world;   
    /// \brief vector to hold the current models present in the world
    std::vector<physics::ModelPtr> m_models;
    /// \brief track model pointer
    physics::ModelPtr m_trackModel;
    /// \brief TFT screen links
    physics::LinkPtr m_tftLink;
    /// \brief variable to define if track components were found
    bool m_trackInitialized;
    
    event::ConnectionPtr m_updateConnection;
    
    gazebo::transport::SubscriberPtr m_stateSub;
    
    int updateState;
    std::string obsModelName, tunModelName;
  };
}
#endif
