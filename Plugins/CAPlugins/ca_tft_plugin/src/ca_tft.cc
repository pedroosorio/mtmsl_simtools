#include "ca_tft.hh"

using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(CATft);

CATft *me;

/// \brief Constructor
CATft::CATft()
{
    me = this;
    m_needUpdate = false;
}

/// \brief deletes allocated memory
CATft::~CATft()
{
 
}

/// \brief Plugin Load function
/// \param _parent - Model pointer to the model defining this plugin
/// \param _sdf - pointer to the SDF of the model
void CATft::Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
    m_parentName = _parent->GetName();
    ROS_WARN("Starting CATft plugin for %s",m_parentName.c_str());
    m_node = transport::NodePtr(new transport::Node());
    m_visual = _parent;
    m_node->Init();
    std::string topic_name = "~/tft/state";
    m_stateSub = m_node->Subscribe(topic_name, &CATft::onStateData, this);
    m_materialName = "CAField/Stop";
    
    std::cout << m_visual->GetMaterialName() << std::endl;
    
    m_updateConnection = event::Events::ConnectPreRender(
    boost::bind(&CATft::onUpdate, this));
}

void CATft::onUpdate()
{
    if(m_needUpdate){
        m_visual->SetMaterial(m_materialName);
        m_visual->Update(); 
        m_needUpdate = false;
    }   
}

void CATft::onStateData(ConstIntPtr &_msg)
{
    std::string new_name = "CAField/Stop";
    switch(_msg->data()){
        case STOP:{
             new_name = "CAField/Stop";
            break;
        }
        case LEFT:{
             new_name = "CAField/Left";
            break;
        }
        case RIGHT:{
             new_name = "CAField/Right";
            break;
        }
        case FORWARD:{
             new_name = "CAField/Forward";
            break;
        }
        case CHECKERS:{
             new_name = "CAField/Checkers";
            break;
        }
        case PARK:{
             new_name = "CAField/Park";
            break;
        }
        default:{
             new_name = "CAField/Stop";
            break;
        }
    }   
    
    m_materialName = new_name; 
    m_needUpdate = true;
}















