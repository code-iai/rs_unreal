#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace uima;


class GetBeliefStateData : public DrawingAnnotator
{
private:
  // Annotator parameters from YAML config. Set defaults here.
  int camera_id_ = 0;
  bool use_hd_images_ = false;

  ros::NodeHandle nh_;

  cv::Mat object_, rgb_;
  cv::Mat rgb_main_cam_; // shall only be used when this annotator runs as a secondary cam 

  image_transport::Publisher image_pub_;
  image_transport::ImageTransport it_;

  enum
  {
    ONLY_RENDER,
    MIXED_WITH_CAMZERO,
    ONLY_OBJECT_MASK,
    TEST_MODE
  } dispMode;
public:
  GetBeliefStateData(): DrawingAnnotator(__func__), nh_("~"), it_(nh_), dispMode(MIXED_WITH_CAMZERO)
  {
    image_pub_ = it_.advertise("beliefstate_image", 5, false);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    if(ctx.isParameterDefined("camera_id"))
      ctx.extractValue("camera_id", camera_id_);
    if(ctx.isParameterDefined("use_hd_images"))
      ctx.extractValue("use_hd_images", use_hd_images_);


    outInfo("Reading camera data from camera id:" << camera_id_);
    outInfo("Use HD Image streams from Main Cam and Belief State? " << use_hd_images_);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);


    sensor_msgs::CameraInfo cam_info;
    cas.get(VIEW_OBJECT_IMAGE, object_, camera_id_);
    cas.get(VIEW_COLOR_IMAGE, rgb_, camera_id_);
    cas.get(VIEW_CAMERA_INFO, cam_info, camera_id_);

    // If we are supposed to mix the rendered image with other image data, 
    // we assume that this camera data is on cam zero
    if(dispMode == MIXED_WITH_CAMZERO)
      cas.get(VIEW_COLOR_IMAGE, rgb_main_cam_, 0);

    // Publish the rendered image to make it available to the outside world
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_).toImageMsg();
    image_pub_.publish(msg);

    outInfo("took: " << clock.getTime() << " ms.");
    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    if(!rgb_.empty())
    {
      switch(dispMode)
      {
        case MIXED_WITH_CAMZERO:
          // std::cout << "Width : " << rgb_.size().width << std::endl;
          // std::cout << "Height: " << rgb_.size().height << std::endl; 
          // std::cout << "Width : " << rgb_main_cam_.size().width << std::endl;
          // std::cout << "Height: " << rgb_main_cam_.size().height << std::endl; 
          // std::cout << "----" << std::endl;
          addWeighted(rgb_, 0.5, rgb_main_cam_, 0.5, 0.0, disp);
        break;
        case ONLY_OBJECT_MASK:
          disp = object_.clone();
        break;
        case TEST_MODE:
          // disp = segmented_object_from_mask_.clone();
        break;
        default:
          disp  = rgb_.clone();
      }
    }
    else
    {
      outInfo("rgb_ is empty. Drawing empty image.");
      disp = cv::Mat::ones(cv::Size(640, 480), CV_8UC3);
    }
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(GetBeliefStateData)