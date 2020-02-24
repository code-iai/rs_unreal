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

bool BeliefStateCommunication::SpawnObject(world_control_msgs::SpawnModel model, float confidence)
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
    BeliefStateCommunication::updateEpisodicMemory(model.response.id,model.request.name,confidence,0);

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

bool  BeliefStateCommunication::isToRotate(world_control_msgs::SpawnModel& model){
     if(model.request.name=="KelloggsCornFlakes")
         return true;
     else
         return false;

}

void  BeliefStateCommunication::rsToUE4ModelMap(world_control_msgs::SpawnModel& model)
{
  if(model.request.name=="KnusperSchokoKeks"){
    model.request.name= "KnusperSchokoKeks";
    model.request.material_names={"KnusperSchoko"};
    model.request.material_paths={"/Models/IAIKitchen/Items/KnusperSchokoKeks"};
    return;
  }

  if(model.request.name=="PfannerGruneIcetea"){
    model.request.name= "PfannerGruneIcetea";
    model.request.material_names={"PfannerGruneIcetea"};
    model.request.material_paths={"/Models/IAIKitchen/Items/PfannerGruneIcetea"};
    return;
  }

  if(model.request.name=="SpitzenReis"){
    model.request.name= "SpitzenReis";
    model.request.material_names={"SpitzenReis"};
    model.request.material_paths={"/Models/IAIKitchen/Items/SpitzenReis"};
    return;
  }

  if(model.request.name=="KoellnMuesliKnusperHonigNuss"){
    model.request.name= "KoellnMuesliKnusperHonigNuss";
    model.request.material_names={"KoellnMuesliKnusperHonigNuss"};
    model.request.material_paths={"/Models/IAIKitchen/Items/KoellnMuesliKnusperHonigNuss"};
    return;
  }

          if(model.request.name=="RedPlasticKnife")
              model.request.name="RedMetalPlateWhiteSpeckles";

         if(model.request.name=="BluePlasticKnife")
              model.request.name="BlueMetalPlateWhiteSpeckles";

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
                                  model.request.name= "Messer";
                                  model.request.material_names={"Plastic_Red"};
                                  model.request.material_paths={"/Items/Materials"};
                              }else{
                                  if(model.request.name=="BluePlasticKnife"){
                                      model.request.name= "Messer";
                                      model.request.material_names={"Material_001"};
                                      model.request.material_paths={"/Items/Messer"};
                                  }else{
                                      if(model.request.name=="BluePlasticFork"){
                                          model.request.name= "DessertFork";
                                          model.request.material_names={"Plastic_Blue"};
                                          model.request.material_paths={"/Items/Materials"};
                                      }else{
                                          if(model.request.name=="RedMetalPlateWhiteSpeckles"){
                                              model.request.name= "RedSpottedPlate1";
                                              model.request.material_names={"Ceramic_Red"};
                                              model.request.material_paths={"/Items/Materials"};
                                          }else{
                                              if(model.request.name=="BlueMetalPlateWhiteSpeckles"){
                                                  model.request.name= "BlueSpottedPlate_1";
                                                  model.request.material_names={"BlueSpottedPlate"};
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
                                                              model.request.name= "SIGG_Flasche";
                                                              model.request.material_names={"Material_0"};
                                                              model.request.material_paths={"/Items/SIGG_Flasche"};
                                                          }else{
                                                              if(model.request.name=="CupEcoOrange"){
                                                                  model.request.name= "Tigercup";
                                                                  model.request.material_names={"Material_0"};
                                                                  model.request.material_paths={"/Items/TigerCup"};
                                                              }else{
                                                                  model.request.name= "ComdoCappuccinoClassico";
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
}

bool BeliefStateCommunication::deleteEpisodicMemory(std::string object_id, std::string object_name, float confidence, int disappeared)
{
  world_control_msgs::DeleteModel model;
  std::map<std::string,std::tuple<std::string,float,int>>::iterator it;
  it=episodic_memory.find(object_id);
  if(it==episodic_memory.end())
      return true;
  int disap=std::get<2>(it->second);
  if(disappeared==1){
     disap++;
     if(disap>1){
            object_name="";
            confidence=10000;
       }else
          return false;
  }else
    disap=0;
  if(std::get<0>(it->second)==object_name)
  {   //improve understanding of an object
      if(std::get<1>(it->second)<confidence){
          episodic_memory.erase(object_id);
          BeliefStateCommunication::updateEpisodicMemory(object_id,object_name,confidence,disap);
      }
     return false;
  }
  //make the belief state more stable
  if(confidence<0.6 || std::get<1>(it->second)>confidence+0.1)
     return false;
  model.request.id=object_id;
  //check whether or not the spawning service server was reached
  if (!delete_client.call(model))
  {
     ROS_ERROR("Failed to call service delete");
     return false;

  }
  //check the status of the respond from the server
  if (!model.response.success)
  {
     ROS_ERROR("Service call returned false");
     return false;
  }
  //print the ID of the spawned hypothesis
  ROS_INFO_STREAM("Object delete with ID " << model.request.id);
 //clear episodic memory
  episodic_memory.erase(object_id);
  ROS_INFO_STREAM("\n finalizing deletion of episodic memory \n ");
  return true;
}

bool BeliefStateCommunication::updateEpisodicMemory(std::string object_id, std::string object_name, float confidence, int disappeared)
{
   ROS_INFO_STREAM("updating episodic memory ...");
   episodic_memory.insert(std::pair<std::string,std::tuple<std::string,float,int>>(object_id,std::tuple<std::string,float,int>(object_name,confidence,disappeared)));
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
