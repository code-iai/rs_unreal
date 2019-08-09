#include <uima/api.hpp>

// Boost
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

//RS
#include <rs/types/all_types.h>
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/io/UnrealVisionBridge.h>
#include <rs/utils/output.h>
#include <rs/utils/common.h>
#include <rs/io/TFBroadcasterWrapper.hpp>
#include <rs/DrawingAnnotator.h>

//ROS
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <robosherlock_msgs/UpdateObjects.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// #include <rapidjson/rapidjson.h>
// #include <rapidjson/document.h>

#include <visualization_msgs/MarkerArray.h>
#include <resource_retriever/retriever.h>

//PCL
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

//C/C++
#include <random>
#include <cmath>
#include <chrono>




using namespace uima;

class GetRenderedView : public DrawingAnnotator
{

private:

  ros::ServiceClient client_;
  ros::NodeHandle nh_;

  UnrealVisionBridge *unrealBridge_;

  image_transport::Publisher image_pub_;
  image_transport::ImageTransport it_;
  cv::Mat object_, rgb_;
  cv::Mat rgb_main_cam_; // shall only be used when this annotator runs as a secondary cam 

  std::thread thread_;
  TFBroadcasterWrapper broadCasterObject_;
  ros::Publisher marker_pub_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr sphereCloud_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr viewCloud_;

  int camera_id_ = 0;

  bool publishAsMarkers_;


  enum
  {
    ONLY_RENDER,
    MIXED_WITH_CAMZERO,
    ONLY_OBJECT_MASK
  } dispMode;

public:
  GetRenderedView(): DrawingAnnotator(__func__), nh_("~"), it_(nh_),publishAsMarkers_(false), dispMode(ONLY_RENDER)
  {
    // client_ = nh_.serviceClient<robosherlock_msgs::UpdateObjects>("/update_objects");
    // TODO make this a parameter so people can parametrize the camera differently, depending on the ip of the UE4 instance
    std::string configFile = ros::package::getPath("rs_unreal") + "/config/config_unreal_vision_localhost.ini";
    outInfo("Reading Unreal Vision config from " << configFile);

    boost::property_tree::ptree pt;
    try{
      boost::property_tree::ini_parser::read_ini(configFile, pt);
    }
    catch(...)
    {
       outError("Couldn't read unreal vision ini file. This will break this annotator");
    }
    ///unrealBridge_ = new UnrealVisionBridge(pt);

    image_pub_ = it_.advertise("rendered_image", 5, false);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("markers", 1, true);
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");

    thread_ = std::thread(&TFBroadcasterWrapper::run, &broadCasterObject_);
    sphereCloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    viewCloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

    if(ctx.isParameterDefined("camera_id"))
        ctx.extractValue("camera_id", camera_id_);

    outInfo("Reading camera data from camera id:" << camera_id_);

    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    delete unrealBridge_;
    return UIMA_ERR_NONE;
  }


  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    MEASURE_TIME;
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();

    std::vector<rs::Plane> planes;
    scene.annotations.filter(planes);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    cas.get(VIEW_CLOUD, *cloud);


    sensor_msgs::CameraInfo cam_info;
    cas.get(VIEW_OBJECT_IMAGE, object_, camera_id_);
    cas.get(VIEW_COLOR_IMAGE, rgb_, camera_id_);
    cas.get(VIEW_CAMERA_INFO, cam_info, camera_id_);

    if(dispMode == MIXED_WITH_CAMZERO)
      cas.get(VIEW_COLOR_IMAGE, rgb_main_cam_, 0);

    std::map<std::string, cv::Vec3b> objectMap;
    cas.get(VIEW_OBJECT_MAP, objectMap, camera_id_);

    // Muesli_2
    // UnrealCV maps contain the ID NAME of the world. Not the actor label name
    //
    try{
      cv::Vec3b muesli_color = objectMap.at("Muesli_2");
      unsigned char blue   = muesli_color.val[0];
      unsigned char green = muesli_color.val[1];
      unsigned char red  = muesli_color.val[2];
      outWarn("Color of muesli: " << ((int)red) << " " << ((int)green) << " " << ((int)blue) );
    }catch(...)
    {
      outError("Exception while catch obj from color map. Maybe the given obj name is not in the mapping");
      return UIMA_ERR_NONE;
    }

    return UIMA_ERR_NONE;
  }


  bool callbackKey(const int key, const Source source)
  {
    switch(key)
    {
    case '1':
      dispMode = ONLY_RENDER;
      outWarn("Switching to ONLY_RENDER mode");
      break;
    case '2':
      dispMode = MIXED_WITH_CAMZERO;
      outWarn("Switching to MIXED WITH CAMZERO");
      break;
    case '3':
      dispMode = ONLY_OBJECT_MASK;
      outWarn("Switching to OBJECT MASK");
      break;
    }

    return true;
  }
  void drawImageWithLock(cv::Mat &disp)
  {
    // disp  = rgb_.clone();
    // return;

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

        default:
          disp  = rgb_.clone();
      }
    }
    else
    {
      disp = cv::Mat::ones(cv::Size(640, 480), CV_8UC3);
    }
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {

    const std::string cloudname = this->name;
    const std::string cloudname2 = "cloudInMap";
    double pointSize = 2.0;
    visualizer.addCoordinateSystem(0.5);
    if(firstRun)
    {
      visualizer.addPointCloud(sphereCloud_, cloudname);
      visualizer.addPointCloud(viewCloud_, cloudname2);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname2);
    }
    else
    {
      visualizer.updatePointCloud(sphereCloud_, cloudname);
      visualizer.updatePointCloud(viewCloud_, cloudname2);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname);
      visualizer.getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pointSize, cloudname2);
    }
  }


};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(GetRenderedView)
