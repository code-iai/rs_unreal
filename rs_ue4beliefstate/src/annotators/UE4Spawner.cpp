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
//RS_UE$BELIEFSTATE
#include "rs_ue4beliefstate/UE4Spawner.h"
  

using namespace uima;

class UE4Spawner : public Annotator
{
private:
  float test_param;
  ros::NodeHandle n;
  ros::ServiceClient client; 


public:

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize parameters");
    //parameter initialization
    client = n.serviceClient<world_control_msgs::SpawnModel>("pie_rwc/spawn_model");
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("reading the belief state ...");
   
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);

    rs::Scene scene = cas.getScene();
    std::vector<rs::ObjectHypothesis> hyps;
    scene.identifiables.filter(hyps);
    outInfo("Found "<<hyps.size()<<" object hypotheses");
    for (auto h:hyps)
    {

      //get the pose, shape and color of each hypothesis
      std::vector<rs::PoseAnnotation>  poses;
      std::vector<rs::Shape> shapes;
      std::vector<rs::SemanticColor> colors;
      h.annotations.filter(shapes);
      h.annotations.filter(colors);
      h.annotations.filter(poses);

      //declare a message for spawning service
      world_control_msgs::SpawnModel srv;

      //set the category of the object to spawn
      srv.request.name = "JaMilch";

      //set the ID of the object object to spawn or automatic generation of ID
      //srv.request.id=argv[2];

      //set the pose of the hypothesis
      srv.request.pose.position.x = poses[0].world.get().translation.get()[0];
      srv.request.pose.position.y = poses[0].world.get().translation.get()[1];
      srv.request.pose.position.z = poses[0].world.get().translation.get()[2];
      srv.request.pose.orientation.x = poses[0].world.get().rotation.get()[0];
      srv.request.pose.orientation.y = poses[0].world.get().rotation.get()[0];
      srv.request.pose.orientation.z = poses[0].world.get().rotation.get()[0];
      srv.request.pose.orientation.w = poses[0].world.get().rotation.get()[0];

      //set the mobility of the hypothesis
      srv.request.physics_properties.mobility = 2; 

      //check whether or not the spawning service server was reached
      if (!client.call(srv))
      {
       ROS_ERROR("Failed to call service client");
       return 1;
      }

      //check the status of the respond from the server
      if (!srv.response.success)
      {
       ROS_ERROR("Service call returned false");
       return 1;
      }

      //print the ID of the spawned hypothesis
      ROS_INFO_STREAM("Object spawned with ID " << srv.response.id);

    }
 
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(UE4Spawner)
