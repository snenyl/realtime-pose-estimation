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

  cv::imshow("Output",cv_image);
  cv::waitKey(33);

//  float width = image.get_width();
//  float height = image.get_height();

//  std::cout << "Running" << width << "\n" << height << std::endl;

}
void PoseEstimation::setup_pose_estimation() {
  p.start();

  std::cout << "Setup" << std::endl;

}
