#include <rs_ue4beliefstate/BeliefStateCommunication.h>
#include <iostream>

BeliefStateCommunication::BeliefStateCommunication() 
{}


BeliefStateCommunication::~BeliefStateCommunication(){} 

bool BeliefStateCommunication::SetCameraPose(geometry_msgs::Pose p)
{
  std::cout << "set camera pose interface test" << std::endl;
  return true;
} 
