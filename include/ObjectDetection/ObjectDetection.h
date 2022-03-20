//
// Created by nylund on 20.03.2022.
//

#ifndef REALTIME_POSE_ESTIMATION_INCLUDE_OBJECTDETECTION_OBJECTDETECTION_H_
#define REALTIME_POSE_ESTIMATION_INCLUDE_OBJECTDETECTION_OBJECTDETECTION_H_

#include "opencv2/opencv.hpp"

class ObjectDetection {
 public:
  void setup_object_detection();
  void run_object_detection(cv::Mat &image);

 private:

};

#endif //REALTIME_POSE_ESTIMATION_INCLUDE_OBJECTDETECTION_OBJECTDETECTION_H_
