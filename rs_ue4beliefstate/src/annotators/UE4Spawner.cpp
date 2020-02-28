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
#include <rs/utils/exception.h>
//RS_UE$BELIEFSTATE
#include "rs_ue4beliefstate/UE4Spawner.h"
  

using namespace uima;

class UE4Spawner : public Annotator
{
private:
  std::string domain="pie_rwc";

  tf::TransformListener listener;
  ros::WallTime start_, end_;
  tf::StampedTransform transform;


public:
  // BeliefStateCommunication* bf_com;

  TyErrorId initialize(AnnotatorContext &ctx)
  {
      outInfo("initializing UE4Spawner annotator ...");
      //parameter initialization
      if(ctx.isParameterDefined("domain"))
      {
           ctx.extractValue("domain", domain);
      }
      //defining the communicator
      // bf_com = new BeliefStateCommunication(domain);

      // BeliefStateAccessor::instance();

      BeliefStateCommunication::belief_changed_in_last_iteration = false;
      return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    // if(bf_com!=nullptr)
          // free(bf_com);
    return UIMA_ERR_NONE;
  }


  void output_transform(tf::Transform transform){
    double yaw, pitch, roll;
    transform.getBasis().getRPY(roll, pitch, yaw);
    
    tf::Quaternion q = transform.getRotation();
    tf::Vector3 v = transform.getOrigin();

    outInfo("- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]");
    outInfo( "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", " 
              << q.getZ() << ", " << q.getW() << "]" << std::endl
              << "            in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl
              << "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI << ", " << yaw*180.0/M_PI << "]"
              );


  }

  TyErrorId process(CAS &tcas, ResultSpecification const &res_spec)
  {
    BeliefStateCommunication::belief_changed_in_last_iteration = false;
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    outInfo("posing the camera in unreal");
    tf::StampedTransform camToWorld, worldToCam;
    tf::StampedTransform ue4ToWorld;

    camToWorld.setIdentity();
    if (scene.viewPoint.has())
    {
      rs::conversion::from(scene.viewPoint.get(), camToWorld);
    }
    else
    {
      outWarn("No camera to world transformation, no further processing!");
      throw rs::Exception("UE4 Spawner doesn't work without a stored viewpoint of the robot camera");
    }
    worldToCam =
        tf::StampedTransform(camToWorld.inverse(), camToWorld.stamp_, camToWorld.child_frame_id_, camToWorld.frame_id_);


    // Please note that the data from the Robot kinect is usually recorded in the rgb optical frame.
    // The UE4 camera equivalent is the rgb LINK frame. So we have to respect this transform also.
    tf::Transform kinect_optical_to_rgb_link;
    tf::Vector3 kinect_optical_to_rgb_link_loc(0,0,0);
    tf::Quaternion kinect_optical_to_rgb_link_rot (-0.5,0.5,-0.5,0.5);
    kinect_optical_to_rgb_link.setOrigin(kinect_optical_to_rgb_link_loc);
    kinect_optical_to_rgb_link.setRotation(kinect_optical_to_rgb_link_rot);

    // HARD CODED UNREAL ENGINE 4 POSE. 
    // THIS COULD BE IN THE ANNOTATOR PARAMETERS
    // OR SEMANTIC MAP?
    tf::Transform worldToUE4;
    tf::Vector3 worldToUE4_loc(0.892, -1.836,0);
    tf::Quaternion worldToUE4_rot (0,0,1,0);
    worldToUE4.setOrigin(worldToUE4_loc);
    worldToUE4.setRotation(worldToUE4_rot);

    // Transform from RGB optical frame coordinates to ue4 world coordinates
    tf::Transform ue4_cam_pose =  worldToUE4 * camToWorld * kinect_optical_to_rgb_link.inverse();

    tf::Transform &ue4trans = ue4_cam_pose;


    geometry_msgs::Pose p;
    p.position.x = ue4trans.getOrigin().getX();
    p.position.y = ue4trans.getOrigin().getY();
    p.position.z = ue4trans.getOrigin().getZ();
    p.orientation.x = ue4trans.getRotation().getX();
    p.orientation.y = ue4trans.getRotation().getY();
    p.orientation.z = ue4trans.getRotation().getZ();
    p.orientation.w = ue4trans.getRotation().getW();

    // bf_com->SetCameraPose(p);
    BeliefStateAccessor::instance()->SetCameraPose(p);



    try{


        outInfo("Current belief state ...");
        BeliefStateAccessor::instance()->printEpisodicMemoryMap();
        // bf_com->printEpisodicMemoryMap();


        long ts_sec=scene.timestamp()/1000000000;
        long ts_nsec=scene.timestamp()%1000000000;
        outInfo("TimeStamp: ******* "<< ts_sec<<" "<<ts_nsec<<" "<<scene.timestamp());
        //look for the transfrom real camera to unreal
        // listener.lookupTransform("/ue4_world","/map", // removed for Mongo DB compatability 
        //                          ros::Time(ts_sec,ts_nsec), transform);

        std::vector<rs::Object> hyps;
        std::vector<rs::MergedHypothesis> mHyps;
        cas.get(VIEW_OBJECTS, hyps);
        scene.identifiables.filter(mHyps);
        outInfo("Found "<<hyps.size()<<" object hypotheses");

        int cluster_id = -1;
        for (auto h:hyps)
        {
          cluster_id++;
          outInfo("OBJ CLUSTER ID: "<<cluster_id << " has identifiable id: " << h.id.get());

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
              // bf_com->deleteEpisodicMemory(h.id.get(),"",100000,1);
              BeliefStateAccessor::instance()->deleteEpisodicMemory(h.id.get(),"",100000,1);
            
          if( !h.inView.get() || h.disappeared.get())
              continue;

          //set the rs category of the hypothesis to spawn
          if(classes.size()>0)
          {

            // outInfo("  Current object name in the belief is: " << bf_com->getCurrentObjectNameForId(h.id.get()));
            outInfo("  Current object name in the belief is: " << BeliefStateAccessor::instance()->getCurrentObjectNameForId(h.id.get()));
            //name
            srv.request.name=classes[0].classname.get();
            //confidence
            confidence=classes[0].confidences.get()[0].score.get();
            outInfo("  Object class and confidence ++++++++++++: " << classes[0].classname.get() << "(" << confidence << ")");
          }else{
            // bf_com->deleteEpisodicMemory(h.id.get(),"",100000,1);
            BeliefStateAccessor::instance()->deleteEpisodicMemory(h.id.get(),"",100000,1);
            outInfo("  No classification on this cluster_id: "<<cluster_id);
            continue;
          }


          // if(!(bf_com->deleteEpisodicMemory(h.id.get(),srv.request.name,confidence,0))){
          if(!(BeliefStateAccessor::instance()->deleteEpisodicMemory(h.id.get(),srv.request.name,confidence,0))){
                outInfo("  DeleteEpisodicMemory failed on: "<<cluster_id);
                continue;
          }

          // outInfo("  OBJ ID ************: "<<h.id.get());
          //set the right category and material of the hypothesis to spawn
          // bf_com->rsToUE4ModelMap(srv);
          BeliefStateAccessor::instance()->rsToUE4ModelMap(srv);

          outInfo("  Mapped Labels to UE4 models. Setting id....");
          //set the ID of the hypothesis to spawn or automatic generation of ID
          srv.request.id=h.id.get();

          // OLD transform from kinect to ue4 (requires TF - can't be used with mongo)

          // {
          // geometry_msgs::PoseStamped p,q;
          // p.header.stamp=ros::Time(ts_sec,ts_nsec);
          // p.header.frame_id="/head_mount_kinect_rgb_optical_frame";
          // p.pose.position.x = poses[0].camera.get().translation.get()[0];
          // p.pose.position.y = poses[0].camera.get().translation.get()[1];
          // p.pose.position.z = poses[0].camera.get().translation.get()[2];
          // p.pose.orientation.x = poses[0].camera.get().rotation.get()[0];
          // p.pose.orientation.y = poses[0].camera.get().rotation.get()[1];
          // p.pose.orientation.z = poses[0].camera.get().rotation.get()[2];
          // p.pose.orientation.w = poses[0].camera.get().rotation.get()[3];
          // //transform to ue4_world
          // listener.transformPose("/ue4_world",p,q);
          // //set the pose of the hypothesis
          // tf::Quaternion r(q.pose.orientation.x,q.pose.orientation.y,q.pose.orientation.z,q.pose.orientation.w);
          // if(bf_com->isToRotate(srv))
          //   r.setRotation(r.getAxis(),r.getAngle()-M_PI/2.0);
          // else
          //   r.setRotation(r.getAxis(),r.getAngle());
          // srv.request.pose.position.x = q.pose.position.x;
          // srv.request.pose.position.y = q.pose.position.y;
          // srv.request.pose.position.z = q.pose.position.z;
          // srv.request.pose.orientation.x = r.getX();
          // srv.request.pose.orientation.y = r.getY();
          // srv.request.pose.orientation.z = r.getZ();
          // srv.request.pose.orientation.w = r.getW();
          // }


          outInfo("  Transform Object....");
          geometry_msgs::Pose p,q;
          p.position.x = poses[0].camera.get().translation.get()[0];
          p.position.y = poses[0].camera.get().translation.get()[1];
          p.position.z = poses[0].camera.get().translation.get()[2];
          p.orientation.x = poses[0].camera.get().rotation.get()[0];
          p.orientation.y = poses[0].camera.get().rotation.get()[1];
          p.orientation.z = poses[0].camera.get().rotation.get()[2];
          p.orientation.w = poses[0].camera.get().rotation.get()[3];

          tf::Transform object_pose;
          tf::Vector3 object_pose_loc(p.position.x, p.position.y, p.position.z);
          tf::Quaternion object_pose_rot (
            p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w);
          object_pose.setOrigin(object_pose_loc);
          object_pose.setRotation(object_pose_rot);

          tf::Transform object_transform_in_ue4 = worldToUE4 * camToWorld * object_pose;
          

          q.position.x = object_transform_in_ue4.getOrigin().getX();
          q.position.y = object_transform_in_ue4.getOrigin().getY();
          q.position.z = object_transform_in_ue4.getOrigin().getZ();
          // TODO Fix rotation fail for cornflakes
          q.orientation.x = object_transform_in_ue4.getRotation().getX();
          q.orientation.y = object_transform_in_ue4.getRotation().getY();
          q.orientation.z = object_transform_in_ue4.getRotation().getZ();
          q.orientation.w = object_transform_in_ue4.getRotation().getW(); 


          tf::Quaternion r(q.orientation.x,q.orientation.y,q.orientation.z,q.orientation.w);
          // if(bf_com->isToRotate(srv))
          if(BeliefStateAccessor::instance()->isToRotate(srv))
            r.setRotation(r.getAxis(),r.getAngle()-M_PI/2.0);
          else
            r.setRotation(r.getAxis(),r.getAngle());
          srv.request.pose.position.x = q.position.x;
          srv.request.pose.position.y = q.position.y;
          srv.request.pose.position.z = q.position.z;
          srv.request.pose.orientation.x = r.getX();
          srv.request.pose.orientation.y = r.getY();
          srv.request.pose.orientation.z = r.getZ();
          srv.request.pose.orientation.w = r.getW();


          //set the mobility of the hypothesis
          srv.request.physics_properties.mobility = 0;
          //spawn hypothesis

          outInfo("  Sending Service request");
          // bf_com->SpawnObject(srv,confidence);
          BeliefStateAccessor::instance()->SpawnObject(srv,confidence);
          outInfo("  Done with Service request");

          BeliefStateCommunication::belief_changed_in_last_iteration = true;

          }catch (Exception ex){
            ROS_ERROR("%s",ex.what());
          }

          }
          }catch (Exception ex){
            ROS_ERROR("%s",ex.what());
          }

    // Allow the rendering stuff to settle
    ros::Duration(1.5).sleep();
    return UIMA_ERR_NONE;
  }
};



    // output_transform(camToWorld * ue4toworld_nonstamped);
    // output_transform(ue4toworld_nonstamped * camToWorld);

    // output_transform(worldToCam * ue4toworld_nonstamped);
    // output_transform(ue4toworld_nonstamped * worldToCam);

    // output_transform(camToWorld * ue4toworld_nonstamped.inverse());
    // output_transform(ue4toworld_nonstamped.inverse() * camToWorld);

    // output_transform(worldToCam * ue4toworld_nonstamped.inverse());
    // output_transform(ue4toworld_nonstamped.inverse() * worldToCam);

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


// This macro exports an entry point that is used to create the annotator.
MAKE_AE(UE4Spawner)
