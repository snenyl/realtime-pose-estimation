//
// Created by nylund on 19.03.2022.
//

#include "PoseEstimation.h"

void PoseEstimation::run_pose_estimation() {
  rs2::frameset frames = p.wait_for_frames();
  rs2::video_frame image = frames.get_color_frame();

  const int w = image.as<rs2::video_frame>().get_width();
  const int h = image.as<rs2::video_frame>().get_height();

  cv::Mat cv_image(cv::Size(w, h), CV_8UC3, (void*)image.get_data(), cv::Mat::AUTO_STEP);
  cv::cvtColor(cv_image,cv_image,cv::COLOR_BGR2RGB);

  image_ = cv_image;

  calculate_aruco();

  cv::imshow("Output",cv_image);
  cv::waitKey(33);


}
void PoseEstimation::setup_pose_estimation() {
  p.start();
  set_camera_parameters();

//  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9);

  std::cout << "Setup" << std::endl;

}
void PoseEstimation::calculate_aruco() {

//  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
//  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
//  std::vector<int> markerIds;

//  cv::aruco::detectMarkers(image_, dictionary_, markerCorners_, markerIds_,
//                           parameters_, rejectedCandidates_);
//
//
//  std::cout << markerCorners_.size() << std::endl;
}

void PoseEstimation::calculate_pose() {

}

void PoseEstimation::set_camera_parameters() {
  camera_matrix_.resize(9);
  dist_coefficients_.resize(5);

  std::fill(camera_matrix_.begin(),camera_matrix_.end(),0);
  std::fill(dist_coefficients_.begin(),dist_coefficients_.end(),0);

  camera_matrix_.at(0) = 907.114; // fx
  camera_matrix_.at(2) = 662.66; // cx
  camera_matrix_.at(4) = 907.605; // fy
  camera_matrix_.at(5) = 367.428; // cy
  camera_matrix_.at(8) = 1; // 1

  dist_coefficients_.at(0) = 0.157553;
  dist_coefficients_.at(1) = -0.501105;
  dist_coefficients_.at(2) = -0.00164696;
  dist_coefficients_.at(3) = 0.000623876;
  dist_coefficients_.at(4) = 0.466404;

//  Color:  [ 1280x720  p[662.66 367.428]  f[907.114 907.605]  Brown Conrady [0.157553 -0.501105 -0.00164696 0.000623876 0.466404] ]
//  Depth:  [ 640x480  p[315.527 255.336]  f[457.488 456.41]  None [0 0 0 0 0] ]
//  camera_matrix_.at(0) =

}
