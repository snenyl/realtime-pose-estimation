//
// Created by nylund on 19.03.2022.
//

#include <iostream>
#include <chrono>
#include <thread>

#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/frustum_culling.h>

#include "ObjectDetection.h"

#ifndef REALTIME_POSE_ESTIMATION_INCLUDE_POSEESTIMATION_H_
#define REALTIME_POSE_ESTIMATION_INCLUDE_POSEESTIMATION_H_

class PoseEstimation {
 public:

  void run_pose_estimation();
  void setup_pose_estimation();

 private:

  //! Aruco functions

  void calculate_aruco();
  void calculate_pose();
  void set_camera_parameters();

  //! Pose estimation functions
  void load_pointcloud();
  void edit_pointcloud();
  void calculate_ransac();
  void calculate_rotation();
  void calculate_pose_vector();
  void calculate_3d_crop();
  pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points);
  void view_pointcloud();

  bool load_from_rosbag = true;



  rs2::pipeline p;
  cv::Mat image_;
  std::string rosbag_path_;

  //! Aruco variables
  std::vector<double> camera_matrix_;
  std::vector<double> dist_coefficients_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_ = getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9);
  cv::Ptr<cv::aruco::DetectorParameters> parameters_ = cv::aruco::DetectorParameters::create();
  std::vector<std::vector<cv::Point2f>> markerCorners_, rejectedCandidates_;
  std::vector<int> markerIds_;

  float example_camera_matrix_data[9] = {907.114,0,662.66,0,907.605,367.428,0,0,1};
  cv::Mat example_camera_matrix_ = cv::Mat(3,3,CV_32F,example_camera_matrix_data);

  float example_dist_coefficients_data[5] = {0.157553,-0.501105,-0.00164696,0.000623876,0.466404};
  cv::Mat example_dist_coefficients_ = cv::Mat(1,5,CV_32F,example_dist_coefficients_data);

  //! Object detection
  ObjectDetection object_detection_object_;

  //! Pose Estimation
    //! Point cloud
    rs2::pointcloud realsense_pointcloud_;
    rs2::points realsense_points_;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_points_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pallet_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;

    std::vector<pcl::PointXYZ> square_frustum_detection_points_;
    pcl::PointXYZ center_frustum_;

    bool first_run_ = true;

    //! Plane_estimation
    std::vector<float> ransac_model_coefficients_;
    pcl::PointXYZ plane_vector_intersect_;

    float fov_v_rad_;
    float fov_h_rad_;


};

#endif //REALTIME_POSE_ESTIMATION_INCLUDE_POSEESTIMATION_H_
