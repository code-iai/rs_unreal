#include <uima/api.hpp>

#include <pcl/point_types.h>
#include <rs/types/all_types.h>
//RS
#include <rs/scene_cas.h>
#include <rs/utils/time.h>
#include <rs/DrawingAnnotator.h>

// Types from this package
#include <rs_unreal/types/all_types.h>

using namespace uima;


class ObjectHyothesisBeliefStateComparison : public DrawingAnnotator
{
private:
  float test_param;
  cv::Mat result_image_;
  cv::Mat rgb_main_cam_; // to compare the real camera data with the belief state in simulation

public:
  ObjectHyothesisBeliefStateComparison(): DrawingAnnotator(__func__)
  {
  }

  TyErrorId initialize(AnnotatorContext &ctx)
  {
    outInfo("initialize");
    ctx.extractValue("test_param", test_param);
    return UIMA_ERR_NONE;
  }

  TyErrorId destroy()
  {
    outInfo("destroy");
    return UIMA_ERR_NONE;
  }
  // cv::Mat getSquareImage( cv::Mat& img, int target_width = 300 )
  void getSquareImage( cv::Mat& img, cv::Mat& out, int target_width = 300 )
  {
    int width = img.cols,
        height = img.rows;

    cv::Mat square = cv::Mat::zeros( target_width, target_width, img.type() );

    int max_dim = ( width >= height ) ? width : height;
    float scale = ( ( float ) target_width ) / max_dim;
    cv::Rect roi;
    if ( width >= height )
    {
      roi.width = target_width;
      roi.x = 0;
      roi.height = height * scale;
      roi.y = ( target_width - roi.height ) / 2;
    }
    else
    {
      roi.y = 0;
      roi.height = target_width;
      roi.width = width * scale;
      roi.x = ( target_width - roi.width ) / 2;
    }

    outWarn("Square ROI: " << roi);

    cv::resize( img, square( roi ), roi.size() );
    square.copyTo(out);
  }

  TyErrorId processWithLock(CAS &tcas, ResultSpecification const &res_spec)
  {
    outInfo("process start");
    MEASURE_TIME;
    rs::StopWatch clock;
    rs::SceneCas cas(tcas);
    rs::Scene scene = cas.getScene();


    cas.get(VIEW_COLOR_IMAGE_HD, rgb_main_cam_, 0);

    // Adjust according to amount of clusters
    result_image_ = cv::Mat(1000, 1000, CV_8UC3, cv::Scalar(0, 100, 150));

    std::vector<rs::ObjectHypothesis> clusters;
    scene.identifiables.filter(clusters);
    int idx = 0;

    int dist_between_clusters_y = 260;
    int image_square_size = 250;

    for (auto cluster:clusters)
    {
      rs::ObjectHypothesis &c = cluster;
      std::vector<rs_unreal::SimBeliefStateObject> bs_object;
      c.annotations.filter(bs_object);

      if(bs_object.size()<1)
      {
        outInfo("No belief state information for cluster "<< idx);
        continue;
      }

      rs_unreal::SimBeliefStateObject& sim_annotation = bs_object[0];
      outInfo("Information for Cluster #" << idx);

      outInfo("  ActorIdName: " << sim_annotation.actorName());

      cv::Rect cv_roi;
      rs::conversion::from(sim_annotation.roi().roi(), cv_roi);
      outWarn("  ROI: "<< cv_roi);

      int red   = sim_annotation.mask_color_r();
      int green = sim_annotation.mask_color_g();
      int blue  = sim_annotation.mask_color_b();
      outInfo("  Mask Color(r g b): " << red << " " << green << " " << blue );

      cv::Mat belief_state_object_crop;
      rs::conversion::from(sim_annotation.rgbImage(), belief_state_object_crop);
      cv::Size s = belief_state_object_crop.size();
      outInfo("  Cropped Belief State Image Dims (width x height): " << s.width << "x" <<  s.height);

      int y_for_cluster_row = idx * dist_between_clusters_y;

      // put the belief state image onto the canvas
      cv::Mat squared_bs;
      getSquareImage(belief_state_object_crop, squared_bs, image_square_size);

      squared_bs.copyTo(
          result_image_(
              cv::Rect(image_square_size, y_for_cluster_row ,squared_bs.cols, squared_bs.rows)
          )
      ); 

      // get the data from the real camera
      rs::ImageROI image_rois = cluster.rois.get();
      cv::Rect real_cluster_roi;
      rs::conversion::from(image_rois.roi_hires(), real_cluster_roi);
      outWarn("  ROI for real object image: "<< real_cluster_roi);
      outInfo("Real img" << rgb_main_cam_.size());

      cv::Mat squared_real_img;
      cv::Mat real_cluster_rgb = rgb_main_cam_(real_cluster_roi);

      getSquareImage(real_cluster_rgb, squared_real_img, image_square_size);

      squared_real_img.copyTo(
          result_image_(
              cv::Rect(0,y_for_cluster_row,squared_real_img.cols, squared_real_img.rows)
          )
      );
      // 
      // rgb_main_cam_(real_cluster_roi).copyTo(
      //     result_image_(
      //         cv::Rect(0,y_for_cluster_row,real_cluster_roi.width, real_cluster_roi.height)
      //     )
      // );
      
      cv::line(result_image_, 
               cv::Point(0,y_for_cluster_row + dist_between_clusters_y - 1),
               cv::Point(result_image_.cols-1, y_for_cluster_row + dist_between_clusters_y -1 ),
               cv::Scalar( 255, 255, 255 )
          );
      
      idx++;

    }
    

    return UIMA_ERR_NONE;
  }

  void drawImageWithLock(cv::Mat &disp)
  {
    disp  = result_image_.clone();
  }

  void fillVisualizerWithLock(pcl::visualization::PCLVisualizer &visualizer, const bool firstRun)
  {

    double pointSize = 2.0;
    visualizer.addCoordinateSystem(0.5);
  }

};

// This macro exports an entry point that is used to create the annotator.
MAKE_AE(ObjectHyothesisBeliefStateComparison)
