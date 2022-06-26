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
  static constexpr char rosbag_relative_path_[] =
      "data/20220327_162128_2meter_with_light_standing_aruco_90_deg_slow_move.bag";  // TODO(simon) Unconst this and implement in configuration file.

  static constexpr uint16_t minimum_object_detection_width_pixels_ = 10;  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr uint16_t minimum_object_detection_height_pixels_ = 10;  // TODO(simon) Unconst this and implement in configuration file.

  static constexpr uint8_t minimum_iterations_before_ransac_ = 10;
  static constexpr uint8_t minimum_ransac_coefficients_ = 3;
  static constexpr uint8_t minimum_marker_corners_ = 0;

  static constexpr uint8_t cv_waitkey_delay_ = 1;
  static constexpr uint8_t pcl_spin_time_ = 1;

  static constexpr uint8_t x_position_id_ = 0;
  static constexpr uint8_t y_position_id_ = 1;
  static constexpr uint8_t z_position_id_ = 2;
  static constexpr uint8_t end_x_position_id_ = 3;
  static constexpr uint8_t end_y_position_id_ = 4;
  static constexpr uint8_t end_z_position_id_ = 5;

  static constexpr uint8_t iterations_start_at_ = 0;

  static constexpr bool realsense_skip_frames_ = false;
  static constexpr float object_detection_nms_threshold_ = 0.3;  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr float object_detection_bbox_conf_threshold_ = 0.1;  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr uint8_t number_of_object_detection_corner_vectors_ = 4;

  static constexpr char object_detection_model_relative_path_[] =
      "models/yolox_s_only_pallet_294epoch_o10/yolox_s_only_pallet_294epoch_o10.xml";  // TODO(simon) Unconst this and implement in configuration file.

  static constexpr char pcl_window_name_[] = "3D Viewer";  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr uint8_t pcl_viewport_id_ = 0;
  static constexpr double pcl_background_color_rgb_[3] = {0, 0, 0};  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr double pcl_initial_camera_position_pos_xyz_view_xyz_up_xyz_[9] = {0, 10, 0, 0, 0, 0, 0, 0, 1};  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr double pcl_coordinate_system_size_ = 1;  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr char top_right_detection_corner_vector_name_[] = "line";
  static constexpr char top_left_detection_corner_vector_name_[] = "line1";
  static constexpr char bottom_right_detection_corner_vector_name_[] = "line2";
  static constexpr char bottom_left_detection_corner_vector_name_[] = "line3";
  static constexpr char center_detection_vector_name_[] = "line_center";
  static constexpr char ground_truth_vector_name_[] = "ground_truth";
  static constexpr char apriltag_coordinate_system_reference_name_[] = "apriltag";
  static constexpr char ground_plane_reference_name_[] = "ground_plane";
  static constexpr char pallet_plane_reference_name_[] = "pallet_plane";
  static constexpr char pose_vector_reference_name_[] = "pose_vector";
  static constexpr char opencv_image_window_name_[] = "Output";

  static constexpr float pcl_frustum_filter_near_plane_distance_meter_ = 0;  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr float pcl_frustum_filter_far_plane_distance_meter_ = 15;  // TODO(simon) Unconst this and implement in configuration file.

  static constexpr uint8_t first_ = 0;
  static constexpr uint8_t second_ = 1;
  static constexpr uint8_t third_ = 2;
  static constexpr uint8_t fourth_ = 3;

//  static constexpr uint8_t k_matrix_fx_id_ = 0;  // TODO(simon) Implement.
//  static constexpr uint8_t k_matrix_fy_id_ = 1;
//  static constexpr uint8_t k_matrix_cx_id_ = 2;
//  static constexpr uint8_t k_matrix_cy_id_ = 3;

  static constexpr uint8_t plane_normal_x_id_ = 0;
  static constexpr uint8_t plane_normal_y_id_ = 1;
  static constexpr uint8_t plane_normal_z_id_ = 2;
  static constexpr uint8_t plane_hessian_component_id_ = 3;

  static constexpr uint8_t red_color_id_ = 0;
  static constexpr uint8_t green_color_id_ = 1;
  static constexpr uint8_t blue_color_id_ = 2;

  static constexpr float rad_to_deg_ = 57.2958;

//  static const cv::Scalar(0, 0, 255) april_tag_marker_color_;// = {0,0,255}; //cv::Scalar(0,0,255);
  static constexpr float april_tag_marker_length_meter_ = 0.535;  // TODO(simon) Unconst this and implement in configuration file.
  pcl::PointXYZ pcl_point_origin_xyz_ = pcl::PointXYZ(0, 0, 0);
  static constexpr double selected_point_color_rgb_[3] = {255,255,0};  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr double center_frustum_vector_color_rgb_[3] = {255, 0, 0};  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr double ground_truth_vector_color_rgb_[3] = {0,0,255};  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr double pose_vector_color_rgb_[3] = {0,255,0};  // TODO(simon) Unconst this and implement in configuration file.

  static constexpr double ransac_eps_angle_radians_ = 0.1;  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr uint16_t ransac_max_iterations_ = 50;  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr double first_ransac_distance_threshold_meter_ = 0.001;  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr double second_ransac_distance_threshold_meter_ = 0.0001;  // TODO(simon) Unconst this and implement in configuration file.

  static constexpr uint16_t sample_surface_normal_sample_size_ = 50;  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr float sample_surface_normal_ratio_ = 0.5;  // TODO(simon) Unconst this and implement in configuration file.

  static constexpr uint16_t minimum_points_for_ransac_ = 10; // TODO(simon) Unconst this and implement in configuration file.
  static constexpr uint16_t minimum_points_for_sampling_surface_normals_ = 10; // TODO(simon) Unconst this and implement in configuration file.

  static constexpr uint16_t maximum_iterations_for_segmentation_ = 500;  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr double segmentation_distance_threshold_meter_ = 0.1;  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr double segmentation_eps_angle_radians_ = 0.1;  // TODO(simon) Unconst this and implement in configuration file.

  static constexpr char logger_file_save_relative_path_[] = "log/data_out.csv";  // TODO(simon) Unconst this and implement in configuration file.
  static constexpr uint64_t debug_print_after_seconds_ = 5;  // TODO(simon) Unconst this and implement in configuration file.

  //! Pallet selection method
  enum pallet_selection_method {  // TODO(simon) Implement pallet selection.
    kMaxConfidence = 0,
    kLargestBoundingBoxSize = 1,
    kLowestDetectionPosition = 2,
    kLowestAndCenterPostition = 3,
  };

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

  bool load_from_rosbag = true;  //! Select if input should be recorder rosbag or direct from camera.  // TODO(simon): Implement in configuration file.
  bool single_run_ = true;  // TODO(simon): Implement in configuration file.
  bool enable_logger_ = true;  // TODO(simon): Implement in configuration file.
  bool enable_debug_mode_ = false;  // TODO(simon): Implement in configuration file.

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

  float example_camera_matrix_data[9] = {907.114, 0, 662.66, 0, 907.605, 367.428, 0, 0, 1};  // TODO(simon) Get K matrix from camera.
  cv::Mat example_camera_matrix_ = cv::Mat(3, 3, CV_32F, example_camera_matrix_data);

  float
      example_dist_coefficients_data[5] = {0.157553, -0.501105, -0.00164696, 0.000623876, 0.466404};  // TODO(simon) Get dist_coefficients from camera.
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
  double zed_k_matrix_[4] = {907.114, 907.605, 662.66,  // TODO(simon) Not full K-matrix.
                             367.428};  // TODO(simon) Get K matrix from camera. This is from Realsense l515. (fx, fy, cx, cy)
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
