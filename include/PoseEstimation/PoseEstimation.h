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



  rs2::pipeline p;
  cv::Mat image_;

  //! Aruco variables

};

#endif //REALTIME_POSE_ESTIMATION_INCLUDE_POSEESTIMATION_H_
