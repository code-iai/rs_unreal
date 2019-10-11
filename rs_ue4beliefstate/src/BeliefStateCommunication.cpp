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

BeliefStateCommunication::BeliefStateCommunication(ros::NodeHandle &nh) : n(nh)
{}

BeliefStateCommunication::BeliefStateCommunication(std::string domain)
{
    outInfo("initializing BeliefStateCommunication ...");
    //parameter initialization
    //client for model spawning
    client = n.serviceClient<world_control_msgs::SpawnModel>(domain+"/spawn_model");
    //client for model deletion
    delete_client = n.serviceClient<world_control_msgs::DeleteModel>(domain+"/delete_model");
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
     ROS_ERROR("Failed to call service spawn");
     return false;
    }

    //check the status of the respond from the server
    if (!model.response.success)
    {
     ROS_ERROR("Spawn Service call returned false");
     return false;
    }

    //save hypothesis in episodic_memory
    BeliefStateCommunication::updateEpisodicMemory(model.response.id);

    //print the ID of the spawned hypothesis
    ROS_INFO_STREAM("Object spawned with ID " << model.response.id);
    return true;
}

bool  BeliefStateCommunication::DeleteObject(world_control_msgs::DeleteModel object_id)
{
    std::cout << "delete hypothesis object_id "<< std::endl;
    return true;
}

bool  BeliefStateCommunication::SetObjectPose(world_control_msgs::SetModelPose pose)
{
    std::cout << "set object pose" << std::endl;
    return true;
}
void  BeliefStateCommunication::rsToUE4ModelMap(world_control_msgs::SpawnModel& model)
{
          if(model.request.name=="KelloggsCornFlakes"){
              model.request.name= "KelloggsCornFlakes";
              model.request.material_names={"KelloggsCornFlakes"};
              model.request.material_paths={"/Models/IAIKitchen/Items/KelloggsCornFlakes"};
          }else{
              if(model.request.name=="VollMilch"){
                  model.request.name= "VollMilch";
                  model.request.material_names={"VollMilch"};
                  model.request.material_paths={"/Models/IAIKitchen/Items/VollMilch"};
              }else{
                  if(model.request.name=="ReineButterMilch"){
                      model.request.name= "ReineButterMilch";
                      model.request.material_names={"ReineButterMilch"};
                      model.request.material_paths={"/Models/IAIKitchen/Items/ReineButterMilch"};
                  }else{
                      if(model.request.name=="MeerSalz"){
                          model.request.name= "MeerSalz";
                          model.request.material_names={"MeerSalz"};
                          model.request.material_paths={"/Models/IAIKitchen/Items/MeerSalz"};
                      }else{
                          if(model.request.name=="PfannerPfirsichIcetea"){
                              model.request.name= "PfannerPfirschIcetea";
                              model.request.material_names={"PfannerPfirschIcetea"};
                              model.request.material_paths={"/Models/IAIKitchen/Items/PfannerPfirschIcetea"};
                          }else{
                              if(model.request.name=="RedPlasticKnife"){
                                  model.request.name= "ButterKnife";
                                  model.request.material_names={"Plastic_Red"};
                                  model.request.material_paths={"/Items/Materials"};
                              }else{
                                  if(model.request.name=="BluePlasticKnife"){
                                      model.request.name= "ButterKnife";
                                      model.request.material_names={"Plastic_Blue"};
                                      model.request.material_paths={"/Items/Materials"};
                                  }else{
                                      if(model.request.name=="BluePlasticFork"){
                                          model.request.name= "DessertFork";
                                          model.request.material_names={"Plastic_Blue"};
                                          model.request.material_paths={"/Items/Materials"};
                                      }else{
                                          if(model.request.name=="RedMetalPlateWhiteSpeckles"){
                                              model.request.name= "ClassicPlate18cm";
                                              model.request.material_names={"Ceramic_Red"};
                                              model.request.material_paths={"/Items/Materials"};
                                          }else{
                                              if(model.request.name=="BlueMetalPlateWhiteSpeckles"){
                                                  model.request.name= "ClassicPlate18cm";
                                                  model.request.material_names={"Ceramic_Blue"};
                                                  model.request.material_paths={"/Items/Materials"};
                                              }else{
                                                  if(model.request.name=="SojaMilch"){
                                                      model.request.name= "SojaMilch";
                                                      model.request.material_names={"SojaMilch"};
                                                      model.request.material_paths={"/Models/IAIKitchen/Items/SojaMilch"};
                                                  }else{
                                                      if(model.request.name=="ElBrygCoffee"){
                                                          model.request.name= "CoffeeElBryg";
                                                          model.request.material_names={"CoffeeElBryg"};
                                                          model.request.material_paths={"/Models/IAIKitchen/Items/CoffeeElBryg"};
                                                      }else{
                                                          if(model.request.name=="SiggBottle"){
                                                              model.request.name= "Mondamin";
                                                              model.request.material_names={"Mondamin"};
                                                              model.request.material_paths={"/Models/IAIKitchen/Items/Mondamin"};
                                                          }else{

                                                                  model.request.name= "Cappuccino";
                                                                  model.request.material_names={"Cappuccino"};
                                                                  model.request.material_paths={"/Models/IAIKitchen/Items/Cappuccino"};

                                                          }
                                                      }
                                                  }
                                              }
                                          }
                                      }
                                  }
                              }
                          }
                      }
                  }
              }
          }
}

bool BeliefStateCommunication::deleteEpisodicMemory()
{
 world_control_msgs::DeleteModel model;
 for (int i=0;i<episodic_memory.size();i++)
 {
      model.request.id=episodic_memory.at(i);
     //check whether or not the spawning service server was reached
     if (!delete_client.call(model))
     {
      ROS_ERROR("Failed to call service delete");
     }

     //check the status of the respond from the server
     if (!model.response.success)
     {
      ROS_ERROR("Service call returned false");
     }

     //print the ID of the spawned hypothesis
     ROS_INFO_STREAM("Object delete with ID " << model.request.id);
 }
 //clear episodic memory
  ROS_INFO_STREAM("\n finalizing deletion of episodic memory \n ");
 episodic_memory.clear();
 return true;
}

bool BeliefStateCommunication::updateEpisodicMemory(std::string object_id)
{
   ROS_INFO_STREAM("updating episodic memory ...");
   episodic_memory.push_back(object_id);
   ROS_INFO_STREAM("finalizing update of episodic memory ...");
   return true;
}


bool BeliefStateCommunication::SetCameraPose(geometry_msgs::Pose p)
{
  ros::ServiceClient setposeclient = n.serviceClient<world_control_msgs::SetModelPose>("pie_rwc/set_model_pose");
  world_control_msgs::SetModelPose setmodelposesrv;
  setmodelposesrv.request.id = "urobovision_camera";
  setmodelposesrv.request.pose = p;
  ROS_INFO_STREAM("Send camera change request");
  setposeclient.call(setmodelposesrv);

  if (!setposeclient.call(setmodelposesrv))
  {
     ROS_ERROR("Failed to call service client while camera update");
     return false;
  }

  if (!setmodelposesrv.response.success)
  {
     ROS_ERROR("Camera Update Service received non-success response during update");
     return false;
  }

  return true;
} 
