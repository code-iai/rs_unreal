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
//ROS
#include <tf/transform_listener.h>
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
  std::string domain="pie_rwc";
  BeliefStateCommunication* bf_com;
  tf::TransformListener listener;
  ros::WallTime start_, end_;


public:

  TyErrorId initialize(AnnotatorContext &ctx)
  {
      outInfo("initializing UE4Spawner annotator ...");
      //parameter initialization
      if(ctx.isParameterDefined("domain"))
      {
           ctx.extractValue("domain", domain);
      }
      //defining the communicator
      bf_com = new BeliefStateCommunication(domain);
      return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    if(bf_com!=nullptr)
          free(bf_com);
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
    //erase episodic memory to prepare update
    bf_com->deleteEpisodicMemory();
    outInfo("Found "<<hyps.size()<<" object hypotheses");
    for (auto h:hyps)
    {

      //get the pose, class, shape and color of each hypothesis
      std::vector<rs::PoseAnnotation>  poses;
      std::vector<rs::Classification>  classes;
      std::vector<rs::Shape> shapes;
      std::vector<rs::SemanticColor> colors;
      h.annotations.filter(shapes);
      h.annotations.filter(colors);
      h.annotations.filter(poses);
      h.annotations.filter(classes);
      //declare a message for spawning service
      world_control_msgs::SpawnModel srv;

      //set the rs category of the hypothesis to spawn
      if(classes.size()>0)
         srv.request.name=classes[0] .classname.get();

      //set the right category and material of the hypothesis to spawn
      bf_com->rsToUE4ModelMap(srv);
      //set the ID of the hypothesis to spawn or automatic generation of ID
      //srv.request.id=argv[2];
      tf::StampedTransform transform;
      //transform from kinect to ue4
      listener.lookupTransform("/ue4_world","/head_mount_kinect_rgb_link", 
                               ros::Time(0), transform);
      geometry_msgs::PoseStamped p,q;
      p.header.frame_id="/head_mount_kinect_rgb_optical_frame";
      p.pose.position.x = poses[0].camera.get().translation.get()[0];
      p.pose.position.y = poses[0].camera.get().translation.get()[1];
      p.pose.position.z = poses[0].camera.get().translation.get()[2];
      p.pose.orientation.x = poses[0].camera.get().rotation.get()[0];
      p.pose.orientation.y = poses[0].camera.get().rotation.get()[1];
      p.pose.orientation.z = poses[0].camera.get().rotation.get()[2];
      p.pose.orientation.w = poses[0].camera.get().rotation.get()[3];
      ROS_ERROR("Hello");
      try{
        listener.transformPose("/ue4_world",p,q);
      }catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
      ROS_ERROR("Hallo");
      //set the pose of the hypothesis
      srv.request.pose.position.x = q.pose.position.x;
      srv.request.pose.position.y = q.pose.position.y;
      srv.request.pose.position.z = q.pose.position.z;
      srv.request.pose.orientation.x = q.pose.orientation.x;
      srv.request.pose.orientation.y = q.pose.orientation.y;
      srv.request.pose.orientation.z = q.pose.orientation.z;
      srv.request.pose.orientation.w = q.pose.orientation.w;

      //set the mobility of the hypothesis
      srv.request.physics_properties.mobility = 0;

      //spawn hypothesis
      bf_com->SpawnObject(srv);


    }
 
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(UE4Spawner)
