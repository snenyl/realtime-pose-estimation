//
// Created by nylund on 19.03.2022.
//

#include <iostream>
#include <chrono>
#include <thread>

#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

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

  // TODO(simon) Need to make Aruco available.
//  cv::Ptr<cv::aruco::Dictionary> dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9);

//  cv::Ptr<cv::aruco::Dictionary> dictionary_ = getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9);


//  cv::Ptr<cv::aruco::DetectorParameters> parameters_;
//  std::vector<std::vector<cv::Point2f>> markerCorners_, rejectedCandidates_;
//  std::vector<int> markerIds_;
//  cv::Ptr<cv::aruco::Dictionary> dictionary_;


};

#endif //REALTIME_POSE_ESTIMATION_INCLUDE_POSEESTIMATION_H_
