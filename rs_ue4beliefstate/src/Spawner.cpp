#include "rs_ue4beliefstate/Spawner.h"
#include <rs_ue4beliefstate/BeliefStateCommunication.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rwc_cpp_client_node");

  ros::NodeHandle n;

  ROS_INFO_STREAM("Testing the BS Communication Lib");
  BeliefStateCommunication bsc;
  bsc.SetCameraPose();

  ros::ServiceClient client = n.serviceClient<world_control_msgs::SpawnModel>("pie_rwc/spawn_model");
  world_control_msgs::SpawnModel srv;
  srv.request.name = "Muesli";
  srv.request.pose.position.x = 1.3;
  srv.request.pose.position.y = -2.1;
  srv.request.pose.position.z = 0.8;
  srv.request.pose.orientation.x = 0.0;
  srv.request.pose.orientation.y = 0.0;
  srv.request.pose.orientation.z = 0.0;
  srv.request.pose.orientation.w = 1.0;
  srv.request.physics_properties.mobility = 2; // set to movable if you plan to update the models pose over time. Otherwise SetModelPose will fail
  //TODO it might be faster to give the exact path for the model
  //to avoid searching for it on the UE4 side


  if (!client.call(srv))
  {
     ROS_ERROR("Failed to call service client");
     return 1;
  }

  if (!srv.response.success)
  {
     ROS_ERROR("Service call returned false");
     return 1;
  }
  ROS_INFO_STREAM("Object spawned with ID " << srv.response.id);

  std::string spawned_object_id = srv.response.id;

  ros::Duration(1.0).sleep();

  ros::ServiceClient setposeclient = n.serviceClient<world_control_msgs::SetModelPose>("pie_rwc/set_model_pose");
  world_control_msgs::SetModelPose setmodelposesrv;
  setmodelposesrv.request.id = spawned_object_id;
  setmodelposesrv.request.pose.position.x = 1.3;
  setmodelposesrv.request.pose.position.y = -2.1;
  setmodelposesrv.request.pose.position.z = 0.85;
  setmodelposesrv.request.pose.orientation.x = 0.0;
  setmodelposesrv.request.pose.orientation.y = 0.0;
  setmodelposesrv.request.pose.orientation.z = 0.0;
  setmodelposesrv.request.pose.orientation.w = 1.0;

  ROS_INFO_STREAM("Changed pose first");
  setposeclient.call(setmodelposesrv);
  ros::Duration(0.5).sleep();

  ros::WallTime start_, end_;




  for (int i = 0; i < 200; i++) {
    start_ = ros::WallTime::now();
    setmodelposesrv.request.pose.position.z += 0.02;
    setposeclient.call(setmodelposesrv);
    // ros::Duration(0.1).sleep();
    end_ = ros::WallTime::now();
    // print results
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
  }

  ros::ServiceClient deletemodelclient = n.serviceClient<world_control_msgs::DeleteModel>("pie_rwc/delete_model");
  world_control_msgs::DeleteModel deletemodelsrv;
  deletemodelsrv.request.id = spawned_object_id;
  ROS_INFO_STREAM("Deleting object");
  deletemodelclient.call(deletemodelsrv);

  return 0;}
