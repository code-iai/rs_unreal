#include <rs_ue4beliefstate/BeliefStateCommunication.h>
#include <iostream>

//UIMA
#include <uima/api.hpp>
//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/boundary.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/console/time.h>

//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/types/all_types.h>



BeliefStateCommunication::BeliefStateCommunication(std::string domain)
{
    outInfo("initializing BeliefStateCommunication ...");
    //parameter initialization
    client = n.serviceClient<world_control_msgs::SpawnModel>(domain+"/spawn_model");
}


BeliefStateCommunication::~BeliefStateCommunication(){
    outInfo("finalizing BeliefStateCommunication ...");
}

bool BeliefStateCommunication::SetCameraPose(world_control_msgs::SetModelPose pose)
{
  std::cout << "set camera pose interface test" << std::endl;
  return true;
} 

bool BeliefStateCommunication::SpawnObject(world_control_msgs::SpawnModel model)
{

    //check whether or not the spawning service server was reached
    if (!client.call(model))
    {
     ROS_ERROR("Failed to call service client");
     return false;
    }

    //check the status of the respond from the server
    if (!model.response.success)
    {
     ROS_ERROR("Service call returned false");
     return false;
    }

    //print the ID of the spawned hypothesis
    ROS_INFO_STREAM("Object spawned with ID " << model.response.id);
    return true;
}

bool DeleteObject(world_control_msgs::DeleteModel object_id)
{
    std::cout << "delete hypothesis object_id "<< std::endl;
    return true;
}

bool SetObjectPose(world_control_msgs::SetModelPose pose)
{
    std::cout << "set object pose" << std::endl;
    return true;
}
