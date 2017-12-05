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

int main(int argc, char **argv)
{
    std::cout << "Track Plugin test program." << std::endl;
    if(argc!=2) {std::cout << "Wrong arguments." << std::endl; return 0;}
    
    while(!gazebo::transport::init());
    std::cout << "Gazebo transport initialized." << std::endl;
    gazebo::transport::run();

    gazebo::transport::NodePtr m_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    if(m_node) m_node->Init();
    else { std::cout << "Failed to create node"; return 0; }
    
    gazebo::transport::PublisherPtr m_statePub = m_node->Advertise<gazebo::msgs::Int>("~/track/obstacle/state");
    gazebo::msgs::Int msg;
    int data = std::stoi(std::string(argv[1]));
    msg.set_data(data);
    
    int packets = 0;
    while(1){ m_statePub->Publish(msg);}
    
   
   

}
