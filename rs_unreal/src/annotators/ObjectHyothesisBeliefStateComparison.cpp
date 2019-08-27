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

    int dist_between_clusters_y = 300;

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

      // cv::Mat croppedRef(object_, roi);
      // // cv::Mat cropped;
      // // Copy the data into new matrix // shouldn't be necessary because rs conv to copies the data
      // // croppedRef.copyTo(cropped);
      // // rs::Mat cropped_rs_mat = rs::create<rs::Mat>(tcas);
      // rs::Mat cropped_rs_mat = rs::conversion::to(tcas,croppedRef);
      // sim_annotation.rgbImage.set(cropped_rs_mat);

      // // Append complete annotation to the cluster
      // c.annotations.append(sim_annotation);
      //
      int y_for_cluster_row = idx * dist_between_clusters_y;

      belief_state_object_crop.copyTo(
          result_image_(
              cv::Rect(400, y_for_cluster_row ,belief_state_object_crop.cols, belief_state_object_crop.rows)
          )
      ); 



      // get the data from the real camera
      rs::ImageROI image_rois = cluster.rois.get();
      cv::Mat real_cluster_rgb;
      cv::Rect real_cluster_roi;
      rs::conversion::from(image_rois.roi_hires(), real_cluster_roi);
      outWarn("  ROI for real object image: "<< real_cluster_roi);
      outInfo("Real img" << rgb_main_cam_.size());

      rgb_main_cam_(real_cluster_roi).copyTo(
          result_image_(
              cv::Rect(0,y_for_cluster_row,real_cluster_roi.width, real_cluster_roi.height)
          )
      );
      
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
