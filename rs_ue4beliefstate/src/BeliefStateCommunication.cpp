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
