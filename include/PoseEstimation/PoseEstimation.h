//
// Created by nylund on 19.03.2022.
//

#include <iostream>
#include <chrono>
#include <thread>

#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

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



  rs2::pipeline p;
  cv::Mat image_;

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

};

#endif //REALTIME_POSE_ESTIMATION_INCLUDE_POSEESTIMATION_H_
