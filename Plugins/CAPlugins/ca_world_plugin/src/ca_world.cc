#include "ca_world.hh"

using namespace gazebo;
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(CAWorld);

/// \brief Constructor
CAWorld::CAWorld()
{
    m_models.clear();
    m_trackInitialized = false;
}

/// \brief deletes allocated memory
CAWorld::~CAWorld()
{
 
}

/// \brief Plugin Load function
/// \param _parent - Model pointer to the model defining this plugin
/// \param _sdf - pointer to the SDF of the model
void CAWorld::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    ROS_WARN("Starting CA World plugin ...");
    m_node = transport::NodePtr(new transport::Node());
    m_world = _parent;
    m_node->Init();

    std::string topic_name = "~/track/obstacle/state";
    m_stateSub = m_node->Subscribe(topic_name, &CAWorld::onStateData, this);
    
    m_updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&CAWorld::onWorldUpdate, this));
}

void CAWorld::onStateData(ConstIntPtr &_msg)
{
    if(updateState!=0) return;
    
    physics::Model_V models = m_world->GetModels();
    int obsModelsFound = 0, tunModelsFound = 0;
    
    for(int i=0;i<models.size();i++){
        if(models[i]->GetName().find("obstacle") != std::string::npos) {
            obsModelsFound++;
            obsModelName = models[i]->GetName();
        }
        
        if(models[i]->GetName().find("tunnel") != std::string::npos) {
            tunModelsFound++;
            tunModelName = models[i]->GetName();
        }
    }
    
    switch(_msg->data()){
        case 0:{
            //m_world->RemoveModel("model://ca_obstacle");
            if(obsModelsFound>0) { ROS_INFO("Obs Remove request"); updateState = 1; }
            //else ROS_WARN("No obstacle in world");
            break;
        }
        case 1:{
            //m_world->InsertModelFile("model://ca_obstacle");
            if(obsModelsFound==0) { ROS_INFO("Obs Add request");  updateState = 2; }
            //else ROS_WARN("Already one obstacle in world");
            break;
        }
        case 2:{
            //m_world->RemoveModel("model://ca_obstacle");
            if(tunModelsFound>0) { ROS_INFO("Tun Remove request"); updateState = 3; }
            //else ROS_WARN("No obstacle in world");
            break;
        }
        case 3:{
            //m_world->InsertModelFile("model://ca_obstacle");
            if(tunModelsFound==0) { ROS_INFO("Tun Add request");  updateState = 4; }
            //else ROS_WARN("Already one obstacle in world");
            break;
        }
        default:{
            break;
        }
    }   
}

void CAWorld::onWorldUpdate()
{
    if(updateState == 1) { m_world->RemoveModel(obsModelName); updateState += 10; return; } 
    else if(updateState == 2) { m_world->InsertModelFile("model://ca_obstacle"); updateState += 10; return; }  
    else if(updateState == 3) { m_world->RemoveModel(tunModelName); updateState += 10; return; } 
    else if(updateState == 4) { m_world->InsertModelFile("model://ca_tunnel"); updateState += 10; return; }
    
    physics::Model_V models = m_world->GetModels();
    int obsModelsFound = 0, tunModelsFound = 0;
    
    for(int i=0;i<models.size();i++){
        if(models[i]->GetName().find("obstacle") != std::string::npos) {
            obsModelsFound++;
        }
        
        if(models[i]->GetName().find("tunnel") != std::string::npos) {
            tunModelsFound++;
        }
    }
    
    if(updateState == 11 && obsModelsFound==0) { updateState = 0; ROS_INFO("Remove completed"); }
    else if(updateState == 12 && obsModelsFound>0) { updateState = 0; ROS_INFO("Add completed"); }
    else if(updateState == 13 && tunModelsFound==0) { updateState = 0; ROS_INFO("Remove completed"); }
    else if(updateState == 14 && tunModelsFound>0) { updateState = 0; ROS_INFO("Add completed"); }

}














