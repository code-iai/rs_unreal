#include "rs_ue4beliefstate/Spawner.h"
#include <tf/transform_listener.h>
#include "geometry_msgs/Pose.h"
#include <rs_ue4beliefstate/BeliefStateCommunication.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rs_ue4beliefstate_camera_updater");

  ros::NodeHandle n;
  tf::TransformListener listener;

  BeliefStateCommunication bsc(n);

  ros::WallTime start_, end_;
  
  // TODO weird error: sometimes the service call will hang
  while(n.ok()){
    start_ = ros::WallTime::now();
    tf::StampedTransform transform;
    try{
      //listener.lookupTransform("/head_mount_kinect_rgb_link", "/ue4_world",  
      listener.lookupTransform("/ue4_world","/head_mount_kinect_rgb_link", 
                               ros::Time(0), transform);
      auto t = transform.getOrigin();
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

      bsc.SetCameraPose(p);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }



    end_ = ros::WallTime::now();
    // print results
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
    ros::Duration(1.5).sleep();
  }

  return 0;
}
