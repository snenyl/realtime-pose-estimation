//
// Created by nylund on 20.03.2022.
//

#ifndef REALTIME_POSE_ESTIMATION_INCLUDE_OBJECTDETECTION_OBJECTDETECTION_H_
#define REALTIME_POSE_ESTIMATION_INCLUDE_OBJECTDETECTION_OBJECTDETECTION_H_

#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <filesystem>

#include <inference_engine.hpp>
#include <opencv2/opencv.hpp>

struct dimensions{
  int width;
  int height;
};

struct Object
{
  cv::Rect_<float> rect;
  int label;
  float prob;
};

struct GridAndStride
{
  int grid0;
  int grid1;
  int stride;
};

class ObjectDetection {
 public:
  void setup_object_detection();
  void run_object_detection(cv::Mat &image);

 private:

  cv::Mat static_resize(cv::Mat& img);
  void blobFromImage(cv::Mat& img, InferenceEngine::Blob::Ptr& blob);
  void decode_outputs(const float* prob, std::vector<Object>& objects, float scale, const int img_w, const int img_h);
  void generate_grids_and_stride(const int target_w, const int target_h, std::vector<int>& strides, std::vector<GridAndStride>& grid_strides);
  void generate_yolox_proposals(std::vector<GridAndStride> grid_strides, const float* feat_ptr, float prob_threshold, std::vector<Object>& objects);
  void qsort_descent_inplace(std::vector<Object> &objects);
  void qsort_descent_inplace(std::vector<Object>& faceobjects, int left, int right);
  void nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked, float nms_threshold);
  inline float intersection_area(const Object& a, const Object& b);
  void draw_objects(const cv::Mat& bgr, const std::vector<Object>& objects);

  //! Settings

  float nms_threshold_;
  float bbox_conf_threshold_;
  int num_classes_;
  dimensions input_dimensions_;
  std::string input_model_path_;
  std::string device_name_;

  //! OpenVino
  InferenceEngine::Core ie_;
  InferenceEngine::CNNNetwork network_;
  InferenceEngine::InputInfo::Ptr input_info_;
  InferenceEngine::DataPtr output_info_;
  InferenceEngine::ExecutableNetwork executable_network_;
  InferenceEngine::InferRequest infer_request_;
//  InferenceEngine::Blob::Ptr output_blob_; // TODO(Simon) Const?
  InferenceEngine::MemoryBlob::CPtr moutput_;
  std::vector<Object> objects_;

  std::string input_name_;
  std::string output_name_;

};

#endif //REALTIME_POSE_ESTIMATION_INCLUDE_OBJECTDETECTION_OBJECTDETECTION_H_
