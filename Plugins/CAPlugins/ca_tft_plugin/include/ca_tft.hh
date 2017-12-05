#ifndef _CA_WORLD_HH_
#define _CA_WORLD_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/int.pb.h>
#include <gazebo/rendering/Visual.hh>
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

typedef enum SignalCode{STOP=0,LEFT,RIGHT,FORWARD,CHECKERS,PARK} SignalCode;

namespace gazebo
{
  /// \brief A plugin to control a soccer robot's omnidirectional movement, kicker and dribbler
  class CATft : public VisualPlugin
  {
    public: 
    /// \brief Constructor
    CATft();

    /// \brief Destructor
    virtual ~CATft();
    
    /// \brief Plugin Load function
    /// \param _parent - Model pointer to the model defining this plugin
    /// \param _sdf - pointer to the SDF of the model
    void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf);
    
    void onUpdate();
    
    inline void setMaterialName(std::string name){ m_materialName = name; }
    
    inline std::string getParentName() { return m_parentName; }
    
    void onStateData(ConstIntPtr &_msg);
    private:
    // VARIABLES
    /// \brief Transport node used to communicate with the transport system
    transport::NodePtr m_node;
    /// \brief pointer to the world object of the simulation
    rendering::VisualPtr m_visual;   
    
    std::string m_parentName, m_materialName;
    
    event::ConnectionPtr m_updateConnection;
    
    gazebo::transport::SubscriberPtr m_stateSub;
    
    bool m_needUpdate;
  };
}
#endif
