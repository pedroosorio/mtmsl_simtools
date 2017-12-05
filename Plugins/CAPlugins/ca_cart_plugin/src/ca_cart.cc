/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "ca_cart.hh"
using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(CACart);

/// \brief Constructor. Initialized deafult variables for various variables
CACart::CACart()
{
    jointsInitialized = false;
}

/// \brief Destructor
CACart::~CACart()
{

}

/// \brief Plugin Load function. Initializes all ros topics for the robot model,
/// also starting message queue thread. Connects gazebo events like world update,
/// time reset and world reset
/// \param _parent - Model pointer to the model defining this plugin
/// \param _sdf - pointer to the SDF of the model
void CACart::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Rename model to specification
    m_model = _parent;
    m_node = transport::NodePtr(new transport::Node());
    m_node->Init();
 
    // Connect server update callback  
    m_updateConn = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&CACart::onUpdate, this));
    
    m_updateConnEnd = event::Events::ConnectWorldUpdateEnd(
    boost::bind(&CACart::updateDirection, this));
    
    m_rrJoint = m_rlJoint = NULL;
    m_rWheel = m_lWheel = NULL;
    
    m_rrJoint = m_model->GetJoint("rr_wheel_joint");
    m_rlJoint = m_model->GetJoint("rl_wheel_joint");
    m_rWheel = m_model->GetLink("fr_wheel::tyre");
    m_lWheel = m_model->GetLink("fl_wheel::tyre");
    
    std::string topic_name = "~/cart/control";
    m_controlSub = m_node->Subscribe(topic_name, &CACart::onControlData, this);
    m_rWheelPose = m_rWheel->GetRelativePose().Ign();
    m_lWheelPose = m_lWheel->GetRelativePose().Ign();
    m_baseRWheelYaw = m_rWheelPose.Rot().Yaw();
    m_baseLWheelYaw = m_lWheelPose.Rot().Yaw();
    
    std::cout << m_rWheelPose << std::endl << m_lWheelPose << std::endl;
    ROS_INFO("Initial Yaws: %.2f %.2f",m_baseRWheelYaw,m_baseLWheelYaw);
    
    velController.Init(10.0,0.0,0.0,0.0,0.0,maxVel,-maxVel);
    dirController.Init(0.01,0.0,0.0,0.0,0.0,maxSteer,-maxSteer);
    
    if(m_rrJoint!=NULL && m_rlJoint!=NULL && m_rWheel!=NULL && m_lWheel!=NULL) {
        jointsInitialized = true;
        ROS_INFO("Cart Joints Initialized");
    } else ROS_ERROR("Error initializing cart joints");
    
    targetVelocity = 0.0;
    targetDirection = 0.0;
}

/// \brief called by event signal every server simulation iteration. Used to reset 
/// parameters like velocities and others
void CACart::onUpdate()
{
    if(jointsInitialized){
        updateVelocity();
    }
}

void CACart::updateVelocity()
{
    common::Time currTime = this->m_model->GetWorld()->GetSimTime();
    common::Time stepTime = currTime - this->m_prevUpdateTime;
    m_prevUpdateTime = currTime;

    if ( stepTime > 0 ) {
        float m_cmd_force = velController.Update(m_rrJoint->GetVelocity(0)-targetVelocity, stepTime);
        m_rrJoint->SetForce(0, m_cmd_force);
   }     
       /* m_rWheelPose = m_rWheel->GetWorldPose().Ign();
    m_lWheelPose = m_lWheel->GetWorldPose().Ign();
    ignition::math::Vector3d r_orient = m_rWheelPose.Rot().Euler();
    m_rWheelPose.Rot().Euler(m_rWheelPose.Rot().Roll(),
                             m_rWheelPose.Rot().Pitch(),
                             3.14159);
                             
    ROS_INFO("%.2f",m_rWheelPose.Rot().Yaw());
    
    m_rWheel->SetWorldPose(m_rWheelPose);
    m_lWheel->SetWorldPose(m_lWheelPose);
    }*/
}

void CACart::updateDirection()
{
        
}
   
void CACart::onControlData(ConstVector2dPtr &_msg)
{
    targetVelocity = _msg->x(); 
    targetDirection = _msg->y()*DEG_TO_RAD; 

}


