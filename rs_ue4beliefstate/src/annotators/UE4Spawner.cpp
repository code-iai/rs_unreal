//Standard C
#include <math.h>
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
  tf::StampedTransform transform;


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

    outInfo("posing the camera in unreal");
   try{
       /* auto t = transform.getOrigin();
        std::cout << t.getX() << " " << t.getY() << " " << t.getZ() << std::endl;
        tf::Quaternion q(
          transform.getRotation().getX(),
          transform.getRotation().getY(),
          transform.getRotation().getZ(),
          transform.getRotation().getW());
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        std::cout << roll << " " << pitch << " " << yaw << std::endl;

        geometry_msgs::Pose p;
        p.position.x = t.getX();
        p.position.y = t.getY();
        p.position.z = t.getZ();
        p.orientation.x = transform.getRotation().getX();
        p.orientation.y = transform.getRotation().getY();
        p.orientation.z = transform.getRotation().getZ();
        p.orientation.w = transform.getRotation().getW();

        bf_com->SetCameraPose(p);
        */

        outInfo("reading the belief state ...");

        rs::StopWatch clock;
        rs::SceneCas cas(tcas);
        rs::Scene scene = cas.getScene();
        long ts_sec=scene.timestamp()/1000000000;
        long ts_nsec=scene.timestamp()%1000000000;
        outInfo("TimeStamp: ******* "<< ts_sec<<" "<<ts_nsec<<" "<<scene.timestamp());
        //look for the transfrom real camera to unreal
        listener.lookupTransform("/ue4_world","/map",
                                 ros::Time(ts_sec,ts_nsec), transform);

        std::vector<rs::Object> hyps;
        std::vector<rs::MergedHypothesis> mHyps;
        cas.get(VIEW_OBJECTS, hyps);
        scene.identifiables.filter(mHyps);
        outInfo("Found "<<hyps.size()<<" object hypotheses");
        for (auto h:hyps)
        {
         try{

          //get the pose, class, shape and color of each hypothesis
          std::vector<rs::PoseAnnotation>  poses;
          std::vector<rs::Classification>  classes;
          std::vector<rs::Shape> shapes;
          std::vector<rs::SemanticColor> colors;
          std::vector<rs::Object> object_ids;
          float confidence=0.0;
          h.annotations.filter(shapes);
          h.annotations.filter(colors);
          h.annotations.filter(poses);
          h.annotations.filter(classes);
          //declare a message for spawning service
          world_control_msgs::SpawnModel srv;

          if(h.inView.get() && (h.disappeared.get()))
              bf_com->deleteEpisodicMemory(h.id.get(),"",100000,1);
          if( !h.inView.get() || h.disappeared.get())
              continue;

          //set the rs category of the hypothesis to spawn
          if(classes.size()>0)
          {
             //name
             srv.request.name=classes[0].classname.get();
             //confidence
             confidence=classes[0].confidences.get()[0].score.get();
             outInfo("OBJ ID SCORE ++++++++++++: "<<confidence);

          }else{
            bf_com->deleteEpisodicMemory(h.id.get(),"",100000,1);
            continue;
          }
          if(!(bf_com->deleteEpisodicMemory(h.id.get(),srv.request.name,confidence,0)))
              continue;
          outInfo("OBJ ID ************: "<<h.id.get());
          //set the right category and material of the hypothesis to spawn
          bf_com->rsToUE4ModelMap(srv);
          //set the ID of the hypothesis to spawn or automatic generation of ID
          srv.request.id=h.id.get();
          geometry_msgs::PoseStamped p,q;
          p.header.stamp=ros::Time(ts_sec,ts_nsec);
          p.header.frame_id="/head_mount_kinect_rgb_optical_frame";
          p.pose.position.x = poses[0].camera.get().translation.get()[0];
          p.pose.position.y = poses[0].camera.get().translation.get()[1];
          p.pose.position.z = poses[0].camera.get().translation.get()[2];
          p.pose.orientation.x = poses[0].camera.get().rotation.get()[0];
          p.pose.orientation.y = poses[0].camera.get().rotation.get()[1];
          p.pose.orientation.z = poses[0].camera.get().rotation.get()[2];
          p.pose.orientation.w = poses[0].camera.get().rotation.get()[3];
          //transform to ue4_world
          listener.transformPose("/ue4_world",p,q);
          //set the pose of the hypothesis
          tf::Quaternion r(q.pose.orientation.x,q.pose.orientation.y,q.pose.orientation.z,q.pose.orientation.w);
          if(bf_com->isToRotate(srv))
            r.setRotation(r.getAxis(),r.getAngle()-M_PI/2.0);
          else
            r.setRotation(r.getAxis(),r.getAngle());
          srv.request.pose.position.x = q.pose.position.x;
          srv.request.pose.position.y = q.pose.position.y;
          srv.request.pose.position.z = q.pose.position.z;
          srv.request.pose.orientation.x = r.getX();
          srv.request.pose.orientation.y = r.getY();
          srv.request.pose.orientation.z = r.getZ();
          srv.request.pose.orientation.w = r.getW();
          //set the mobility of the hypothesis
          srv.request.physics_properties.mobility = 0;
          //spawn hypothesis
          bf_com->SpawnObject(srv,confidence);
        }catch (Exception ex){
          ROS_ERROR("%s",ex.what());
        }

    }
   }catch (Exception ex){
            ROS_ERROR("%s",ex.what());
   }
 
  }
};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(UE4Spawner)
