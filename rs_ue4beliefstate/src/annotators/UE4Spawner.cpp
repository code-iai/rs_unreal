#include <uima/api.hpp>
#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include "rs_ue4beliefstate/UE4Spawner.h"
  

using namespace uima;

class UE4Spawner : public Annotator
{
private:
  float test_param;

public:

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize parameters");
    //parameter initialization
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<world_control_msgs::SpawnModel>("pie_rwc/spawn_model");
    world_control_msgs::SpawnModel srv;
    srv.request.name = "JaMilch";
    //srv.request.id=argv[2];
    srv.request.pose.position.x = 1;
    srv.request.pose.position.y = 1;
    srv.request.pose.position.z = 1;
    srv.request.pose.orientation.x = 0.0;
    srv.request.pose.orientation.y = 0.0;
    srv.request.pose.orientation.z = 0.0;
    srv.request.pose.orientation.w = 1.0;

    srv.request.physics_properties.mobility = 2; 

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

 
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(UE4Spawner)
