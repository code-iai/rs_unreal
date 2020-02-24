#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/TransformStamped.h"


// This script transforms a frame frameA into a frame frameB and publishes this as 
// on a seperate Topic outputTopic as a TF2 message
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rs_ue4beliefstate_pr2_base_updater");

  std::string outputTopic = "/tf_replay";
  std::string frameA = "/base_footprint"; 
  std::string frameB = "/map";

  ros::NodeHandle n;
  tf::TransformListener listener;
  ros::Publisher base_link_frame_pub = n.advertise<tf2_msgs::TFMessage>(outputTopic, 1000);

  ros::WallTime start_, end_;
  
  while(n.ok()){
    start_ = ros::WallTime::now();
    tf::StampedTransform transform;
    try{
      //listener.lookupTransform("/map","/base_footprint",
      listener.lookupTransform(frameB,frameA,
                               ros::Time(0), transform);

      {
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
      }

      // Fix/Offset base_footprint frame for michael
      
      auto location = transform.getOrigin();
      location.setZ(location.getZ() + 0.071);
      transform.setOrigin(location);
     
      {
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
      }


      tf2_msgs::TFMessage tf_msg;
      geometry_msgs::TransformStamped transform_stamped_srcframe_in_map;
      tf::transformStampedTFToMsg(transform, transform_stamped_srcframe_in_map);

      tf_msg.transforms.push_back(transform_stamped_srcframe_in_map);
      base_link_frame_pub.publish(tf_msg);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      continue;
    }



    end_ = ros::WallTime::now();
    // print results
    double execution_time = (end_ - start_).toNSec() * 1e-6;
    ROS_INFO_STREAM("Exectution time (ms): " << execution_time);
    ros::Duration(0.05).sleep();
  }

  return 0;
}
