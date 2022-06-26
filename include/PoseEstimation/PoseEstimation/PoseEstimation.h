// Copyright 2022 Simon Erik Nylund.
// Author: snenyl

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/sampling_surface_normal.h>
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/frustum_culling.h>
#include <pcl/io/pcd_io.h>
#include <jsoncpp/json/json.h>

#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>
#include <vector>

#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

#include "ObjectDetection/ObjectDetection.h"

#ifndef INCLUDE_POSEESTIMATION_POSEESTIMATION_POSEESTIMATION_H_
#define INCLUDE_POSEESTIMATION_POSEESTIMATION_POSEESTIMATION_H_

class PoseEstimation {  // TODO(simon) Add Doxygen documentation.
 public:
  void run_pose_estimation();

  void setup_pose_estimation();

 private:
  //! Variables
  // TODO(simon) Add magic numbers from PoseEstimation.cc here with "static constexpr" as prefix.
  static constexpr uint8_t x_position_ = 0;
  static constexpr uint8_t y_position_ = 1;
  static constexpr uint8_t z_position_ = 2;

  static constexpr uint8_t real_time_ = 0;

  static constexpr uint8_t first_ = 0;
  static constexpr uint8_t second_ = 1;
  static constexpr uint8_t third_ = 2;
  static constexpr uint8_t fourth_ = 3;
  
  static constexpr uint8_t zero_ = 0;

//  static const auto april_tag_marker_color_ = cv::Scalar(0,0,255);
  static constexpr float april_tag_marker_length_ = 0.535;  // TODO(simon) Unconst this and implement in configuration file.
  pcl::PointXYZ origo_ = pcl::PointXYZ(0,0,0);
  
  static constexpr double ransac_eps_angle_radians_ = 0.1;  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr uint16_t ransac_max_iterations_ = 50;  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr double first_ransac_distance_threshold_meter_ = 0.001;  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr double second_ransac_distance_threshold_meter_ = 0.0001;  // TODO(simon) Unconst this and implement in configuration file.

  static constexpr uint16_t sample_surface_normal_sample_size_ = 50;  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr float sample_surface_normal_ratio_ = 0.5;  // TODO(simon) Unconst this and implement in configuration file.

  static constexpr uint16_t minimum_points_for_ransac_ = 10;

  static constexpr uint16_t maximum_iterations_for_segmentation_ = 500;  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr double segmentation_distance_threshold_meter_ = 0.1;  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr double segmentation_eps_angle_radians_ = 0.1;  // TODO(simon) Unconst this and implement in configuration file.

  //! Aruco functions
  void calculate_aruco();

  void calculate_pose();

  void set_camera_parameters();

  //! Pose estimation functions
  void edit_pointcloud();

  void calculate_ransac();

  void calculate_pose_vector();

  void calculate_3d_crop();

  pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points &points);

  void view_pointcloud();

  void log_data(uint32_t frame);

  bool load_from_rosbag = true;  //! Select if input should be recorder rosbag or direct from camera.
  bool single_run_ = true;
  bool enable_logger_ = true;
  bool enable_debug_mode_ = false;

  //! Camera
  rs2::pipeline p;
  cv::Mat image_;
  std::string rosbag_path_;

  //! Aruco variables
  std::vector<double> camera_matrix_;
  std::vector<double> dist_coefficients_;

  cv::Ptr<cv::aruco::Dictionary>
      dictionary_ = getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9);
  cv::Ptr<cv::aruco::DetectorParameters> parameters_ = cv::aruco::DetectorParameters::create();
  std::vector<std::vector<cv::Point2f>> markerCorners_, rejectedCandidates_;
  std::vector<int> markerIds_;

  float example_camera_matrix_data[9] = {907.114, 0, 662.66, 0, 907.605, 367.428, 0, 0, 1};
  cv::Mat example_camera_matrix_ = cv::Mat(3, 3, CV_32F, example_camera_matrix_data);

  float
      example_dist_coefficients_data[5] = {0.157553, -0.501105, -0.00164696, 0.000623876, 0.466404};
  cv::Mat example_dist_coefficients_ = cv::Mat(1, 5, CV_32F, example_dist_coefficients_data);

  std::vector<cv::Vec<double, 3>> rvecs_;
  std::vector<cv::Vec<double, 3>> tvecs_;

  std::vector<double> rotation_deg_;

  Eigen::Affine3f rot_trans_matrix_;
  Eigen::Affine3f create_rotation_matrix(float ax, float ay, float az);

  cv::Mat ground_truth_vector_;

  std::vector<double> converted_ground_truth_vector_ = {0, 0, 0, 0, 0, 0};

  //! Object detection
  ObjectDetection object_detection_object_;
  ObjectDetection pallet_void_object_detection_object_;

  object_detection_output detection_output_struct_;
  object_detection_output pallet_void_detection_output_struct_;

  //! Pose Estimation
  //! Point cloud
  rs2::pointcloud realsense_pointcloud_;
  rs2::points realsense_points_;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_points_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pallet_;
  pcl::PointCloud<pcl::PointNormal>::Ptr output_cloud_with_normals_;
  pcl::PointCloud<pcl::PointNormal>::Ptr extracted_cloud_with_normals_;
  pcl::PointCloud<pcl::PointNormal>::Ptr final_with_normals_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_;
  pcl::visualization::PCLVisualizer::Ptr viewer_;

  std::vector<pcl::PointXYZ> square_frustum_detection_points_;
  pcl::PointXYZ center_frustum_;

  bool first_run_ = true;

  //! Plane_estimation
  std::vector<float> ransac_model_coefficients_;
  std::vector<float> first_ransac_model_coefficients_;
  std::vector<float> second_ransac_model_coefficients_;
  pcl::PointIndices::Ptr inliers_;
  std::vector<int> frustum_filter_inliers_;
  double zed_k_matrix_[4] = {907.114, 907.605, 662.66,
                             367.428};  // TODO(simon) Get from camera. This is from Realsense l515. (fx, fy, cx, cy)
  std::vector<Eigen::Vector2d> detection_from_image_center_;
  double detection_vector_scale_ = 3;
  float fov_v_rad_;

  float fov_h_rad_;
  //! Time
  std::chrono::time_point<std::chrono::system_clock>
      start_debug_time_ = std::chrono::system_clock::now();

  int wait_with_ransac_for_ = 0;

  //! Pose vector:
  pcl::ModelCoefficients intersect_point_;
  pcl::PointXYZ plane_vector_intersect_;
  pcl::PointXYZ plane_frustum_vector_intersect_;
  pcl::PointXYZ pose_vector_end_point_;
};

#endif  // INCLUDE_POSEESTIMATION_POSEESTIMATION_POSEESTIMATION_H_
