// Copyright 2022 Simon Erik Nylund.
// Author: snenyl

#ifndef INCLUDE_OBJECTDETECTION_OBJECTDETECTION_OBJECTDETECTION_H_
#define INCLUDE_OBJECTDETECTION_OBJECTDETECTION_OBJECTDETECTION_H_

#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <utility>

#include <inference_engine.hpp>
#include <opencv2/opencv.hpp>

struct dimensions {
  int width;
  int height;
};

struct Object {
  cv::Rect_<float> rect;
  int label;
  float prob;
};

struct GridAndStride {
  int grid0;
  int grid1;
  int stride;
};

struct object_detection_output {
  uint16_t x;
  uint16_t y;
  uint16_t width;
  uint16_t height;
  double confidence;
};

class ObjectDetection {  // TODO(simon) Add Doxygen documentation.
 public:
  void setup_object_detection();

  void run_object_detection(cv::Mat &image);  // TODO(simon) Check if this is a non-const reference. If so, make const or use a pointer.

  void set_model_path(std::string path);

  void set_object_detection_settings(float nms_threshold, float bbox_conf_threshold);

  object_detection_output get_detection();

 private:   // TODO(simon) Add magic numbers from ObjectDetection.cc here with "static constexpr" as prefix.
  cv::Mat static_resize(cv::Mat &img);  // TODO(simon) Check if this is a non-const reference. If so, make const or use a pointer.

  void blobFromImage(cv::Mat &img,
                     InferenceEngine::Blob::Ptr &blob);  // TODO(simon) Check if this is a non-const reference. If so, make const or use a pointer.

  void decode_outputs(const float *prob,
                      std::vector<Object> &objects,  // TODO(simon) Check if this is a non-const reference. If so, make const or use a pointer.
                      float scale,
                      const int img_w,
                      const int img_h);

  void generate_grids_and_stride(const int target_w,
                                 const int target_h,
                                 std::vector<int> &strides,  // TODO(simon) Check if this is a non-const reference. If so, make const or use a pointer.
                                 std::vector<GridAndStride> &grid_strides);  // TODO(simon) Check if this is a non-const reference. If so, make const or use a pointer.

  void generate_yolox_proposals(std::vector<GridAndStride> grid_strides,
                                const float *feat_ptr,
                                float prob_threshold,
                                std::vector<Object> &objects);  // TODO(simon) Check if this is a non-const reference. If so, make const or use a pointer.

  void qsort_descent_inplace(std::vector<Object> &objects);  // TODO(simon) Check if this is a non-const reference. If so, make const or use a pointer.

  void qsort_descent_inplace(std::vector<Object> &faceobjects,
                             int left,
                             int right);  // TODO(simon) Check if this is a non-const reference. If so, make const or use a pointer.

  void nms_sorted_bboxes(const std::vector<Object> &faceobjects,
                         std::vector<int> &picked,  // TODO(simon) Check if this is a non-const reference. If so, make const or use a pointer.
                         float nms_threshold);

  inline float intersection_area(const Object &a, const Object &b);

  void draw_objects(const cv::Mat &bgr, const std::vector<Object> &objects);

  double box_filtering(double image_width,
                       double image_height,
                       std::vector<object_detection_output> detection,
                       uint16_t i);

  //! Settings

  float nms_threshold_;
  float bbox_conf_threshold_;
  int num_classes_;
  dimensions input_dimensions_;
  std::string model_path_;
  std::string input_model_path_;
  std::string device_name_;

  //! OpenVino
  InferenceEngine::Core ie_;
  InferenceEngine::CNNNetwork network_;
  InferenceEngine::InputInfo::Ptr input_info_;
  InferenceEngine::DataPtr output_info_;
  InferenceEngine::ExecutableNetwork executable_network_;
  InferenceEngine::InferRequest infer_request_;
  InferenceEngine::MemoryBlob::CPtr moutput_;
  std::vector<Object> objects_;

  std::string input_name_;
  std::string output_name_;

  std::vector<object_detection_output> detection_output_struct_;
};

#endif  // INCLUDE_OBJECTDETECTION_OBJECTDETECTION_OBJECTDETECTION_H_
