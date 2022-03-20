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
  calculate_pose();

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

  cv::aruco::detectMarkers(image_, dictionary_, markerCorners_, markerIds_, parameters_, rejectedCandidates_);


  if (markerCorners_.size() > 0){
    cv::drawMarker(image_,markerCorners_.at(0).at(0),cv::Scalar(0,0,255));
    cv::drawMarker(image_,markerCorners_.at(0).at(1),cv::Scalar(0,0,255));
    cv::drawMarker(image_,markerCorners_.at(0).at(2),cv::Scalar(0,0,255));
    cv::drawMarker(image_,markerCorners_.at(0).at(3),cv::Scalar(0,0,255));
      }



//  cv::aruco::detectMarkers(image_, dictionary_, markerCorners_, markerIds_,
//                           parameters_, rejectedCandidates_);
//
//
//  std::cout << markerCorners_.size() << std::endl;
}

void PoseEstimation::calculate_pose() {

  std::vector<cv::Vec3d> rvecs, tvecs, object_points;
  cv::aruco::estimatePoseSingleMarkers(markerCorners_,0.535,example_camera_matrix_,example_dist_coefficients_,rvecs,tvecs,object_points);
  // TODO(simon) Set marker size as parameter. 0.175 0.535

//  std::cout << rvecs.size() << "\n" << tvecs.size() << "\n" << object_points.size() << std::endl;
//  std::cout << rvecs.at(0) << "\n" << tvecs.at(0) << std::endl;

  if (rvecs.size() > 0 && tvecs.size() > 0){
    std::stringstream rotation;
    std::stringstream translation;

    rotation << rvecs.at(0);
    translation << tvecs.at(0);
//    std::cout << rvecs.size() << "\n" << tvecs.size() << "\n" << object_points.size() << std::endl;
//    std::cout << rvecs.at(0) << "\n" << tvecs.at(0) << std::endl;
    cv::putText(image_,rotation.str(),cv::Point(50,50),cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,255,0),2,false);
    cv::putText(image_,translation.str(),cv::Point(50,100),cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,255,0),2,false);
    cv::aruco::drawAxis(image_,example_camera_matrix_,example_dist_coefficients_,rvecs,tvecs,0.1);
  }

//  cv::aruco::drawAxis(image_,example_camera_matrix,example_dist_coefficients,rvecs,tvecs,0.1);

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
