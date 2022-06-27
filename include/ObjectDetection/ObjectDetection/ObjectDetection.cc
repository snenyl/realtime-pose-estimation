// Copyright 2022 Simon Erik Nylund.
// Author: snenyl

#include "ObjectDetection/ObjectDetection.h"

void ObjectDetection::setup_object_detection() {
  input_model_path_ = std::filesystem::current_path().parent_path() / model_path_;
  num_classes_ = number_of_classes_;
  input_dimensions_.width = network_input_dimensions_wh_[width_id_];
  input_dimensions_.height = network_input_dimensions_wh_[height_id_];
  device_name_ = inference_device_name_;

  std::cout << input_model_path_ << std::endl;

  network_ = ie_.ReadNetwork(input_model_path_);

  if (network_.getOutputsInfo().size() != 1)  // TODO(simon) Magic number.
    std::cout << "Sample supports topologies with 1 output only" << std::endl;
  if (network_.getInputsInfo().size() != 1)  // TODO(simon) Magic number.
    std::cout << "Sample supports topologies with 1 input only" << std::endl;

  input_info_ = network_.getInputsInfo().begin()->second;
  input_name_ = network_.getInputsInfo().begin()->first;

  if (network_.getOutputsInfo().empty()) {
    std::cerr << "Network outputs info is empty" << std::endl;
  }

  output_info_ = network_.getOutputsInfo().begin()->second;
  output_name_ = network_.getOutputsInfo().begin()->first;

  executable_network_ = ie_.LoadNetwork(network_, device_name_);
  infer_request_ = executable_network_.CreateInferRequest();
}

void ObjectDetection::run_object_detection(cv::Mat &image) {
  cv::Mat pr_image = static_resize(image);
  InferenceEngine::Blob::Ptr imgBlob = infer_request_.GetBlob(input_name_);
  blobFromImage(pr_image, imgBlob);

  infer_request_.Infer();

  const InferenceEngine::Blob::Ptr output_blob = infer_request_.GetBlob(output_name_);
  moutput_ = InferenceEngine::as<InferenceEngine::MemoryBlob>(output_blob);
  if (!moutput_) {
    std::cout << "We expect output to be inherited from MemoryBlob, "
                 "but by fact we were not able to cast output to MemoryBlob" << std::endl;
  }

  auto moutputHolder = moutput_->rmap();
  const auto *net_pred =
      moutputHolder.as<const InferenceEngine::PrecisionTrait<InferenceEngine::Precision::FP32>::value_type *>();

  int img_w = image.cols;
  int img_h = image.rows;

  float scale = std::min(input_dimensions_.width / (image.cols * 1.0),
                         input_dimensions_.height
                             / (image.rows * 1.0));  // TODO(simon) Magic number.

  decode_outputs(net_pred, objects_, scale, img_w, img_h);
  draw_objects(image, objects_);
}

cv::Mat ObjectDetection::static_resize(cv::Mat &img) {
  float r = std::min(input_dimensions_.width / (img.cols * 1.0),
                     input_dimensions_.height / (img.rows * 1.0));  // TODO(simon) Magic number.
  int unpad_w = r * img.cols;
  int unpad_h = r * img.rows;
  cv::Mat re(unpad_h, unpad_w, CV_8UC3);
  cv::resize(img, re, re.size());
  cv::Mat out(input_dimensions_.height,
              input_dimensions_.width,
              CV_8UC3,
              cv::Scalar(114, 114, 114));  // TODO(simon) Magic number.
  re.copyTo(out(cv::Rect(0, 0, re.cols, re.rows)));
  return out;
}

void ObjectDetection::blobFromImage(cv::Mat &img, InferenceEngine::Blob::Ptr &blob) {
  int channels = 3;  // TODO(simon) Magic number.
  int img_h = img.rows;
  int img_w = img.cols;
  InferenceEngine::MemoryBlob::Ptr mblob = InferenceEngine::as<InferenceEngine::MemoryBlob>(blob);
  if (!mblob) {
    std::cout << "We expect blob to be inherited from MemoryBlob in matU8ToBlob, "
              << "but by fact we were not able to cast inputBlob to MemoryBlob" << std::endl;
  }
  // locked memory holder should be alive all time while access to its buffer happens
  auto mblobHolder = mblob->wmap();

  float *blob_data = mblobHolder.as<float *>();

  for (size_t c = 0; c < channels; c++) {  // TODO(simon) Magic number.
    for (size_t h = 0; h < img_h; h++) {  // TODO(simon) Magic number.
      for (size_t w = 0; w < img_w; w++) {  // TODO(simon) Magic number.
        blob_data[c * img_w * img_h + h * img_w + w] =
            static_cast<float>(img.at<cv::Vec3b>(h, w)[c]);
      }
    }
  }
}
void ObjectDetection::decode_outputs(const float *prob,
                                     std::vector<Object> &objects,
                                     float scale,
                                     const int img_w,
                                     const int img_h) {
  std::vector<Object> proposals;
  std::vector<int> strides = inference_stride_;
  std::vector<GridAndStride> grid_strides;

  generate_grids_and_stride(input_dimensions_.width,
                            input_dimensions_.height,
                            strides,
                            grid_strides);
  generate_yolox_proposals(grid_strides, prob, bbox_conf_threshold_, proposals);

  if (proposals.size() > 0) {
    qsort_descent_inplace(proposals,
                          0,
                          0);  // TODO(simon) Getting: "Process finished with exit code 139 (interrupted by signal 11: SIGSEGV)"  // TODO(simon) Magic number.
  }

  std::vector<int> picked;
  nms_sorted_bboxes(proposals, picked, nms_threshold_);
  int count = picked.size();
  objects.resize(count);

  for (int i = 0; i < count; i++) {   // TODO(simon) Magic number.
    objects[i] = proposals[picked[i]];

    // adjust offset to original unpadded
    float x0 = (objects[i].rect.x) / scale;
    float y0 = (objects[i].rect.y) / scale;
    float x1 = (objects[i].rect.x + objects[i].rect.width) / scale;
    float y1 = (objects[i].rect.y + objects[i].rect.height) / scale;

    // clip
    x0 = std::max(std::min(x0, static_cast<float>((img_w - 1))), 0.f);  // TODO(simon) Magic number.
    y0 = std::max(std::min(y0, static_cast<float>((img_h - 1))), 0.f);  // TODO(simon) Magic number.
    x1 = std::max(std::min(x1, static_cast<float>((img_w - 1))), 0.f);  // TODO(simon) Magic number.
    y1 = std::max(std::min(y1, static_cast<float>((img_h - 1))), 0.f);  // TODO(simon) Magic number.

    objects[i].rect.x = x0;
    objects[i].rect.y = y0;
    objects[i].rect.width = x1 - x0;
    objects[i].rect.height = y1 - y0;
  }
}

void ObjectDetection::generate_grids_and_stride(const int target_w,
                                                const int target_h,
                                                std::vector<int> &strides,
                                                std::vector<GridAndStride> &grid_strides) {
  for (auto stride : strides) {
    int num_grid_w = target_w / stride;
    int num_grid_h = target_h / stride;
    for (int g1 = 0; g1 < num_grid_h; g1++) {  // TODO(simon) Magic number.
      for (int g0 = 0; g0 < num_grid_w; g0++) {  // TODO(simon) Magic number.
        grid_strides.push_back((GridAndStride) {g0, g1, stride});
      }
    }
  }
}

void ObjectDetection::generate_yolox_proposals(std::vector<GridAndStride> grid_strides,
                                               const float *feat_ptr,
                                               float prob_threshold,
                                               std::vector<Object> &objects) {
  const int num_anchors = grid_strides.size();

  for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++) {  // TODO(simon) Magic number.
    const int grid0 = grid_strides[anchor_idx].grid0;
    const int grid1 = grid_strides[anchor_idx].grid1;
    const int stride = grid_strides[anchor_idx].stride;

    const int basic_pos = anchor_idx * (num_classes_ + 5);  // TODO(simon) Magic number.

    // yolox/models/yolo_head.py decode logic
    //  outputs[..., :2] = (outputs[..., :2] + grids) * strides
    //  outputs[..., 2:4] = torch.exp(outputs[..., 2:4]) * strides
    float x_center = (feat_ptr[basic_pos + 0] + grid0) * stride;  // TODO(simon) Magic number.
    float y_center = (feat_ptr[basic_pos + 1] + grid1) * stride;  // TODO(simon) Magic number.
    float w = exp(feat_ptr[basic_pos + 2]) * stride;  // TODO(simon) Magic number.
    float h = exp(feat_ptr[basic_pos + 3]) * stride;  // TODO(simon) Magic number.
    float x0 = x_center - w * 0.5f;  // TODO(simon) Magic number.
    float y0 = y_center - h * 0.5f;  // TODO(simon) Magic number.

    float box_objectness = feat_ptr[basic_pos + 4];  // TODO(simon) Magic number.
    for (int class_idx = 0; class_idx < num_classes_; class_idx++) {  // TODO(simon) Magic number.
      float box_cls_score = feat_ptr[basic_pos + 5 + class_idx];  // TODO(simon) Magic number.
      float box_prob = box_objectness * box_cls_score;
      if (box_prob > prob_threshold) {
        Object obj;
        obj.rect.x = x0;
        obj.rect.y = y0;
        obj.rect.width = w;
        obj.rect.height = h;
        obj.label = class_idx;
        obj.prob = box_prob;

        objects.emplace_back(obj);
      }
    }  // class loop
  }  // point anchor loop
}
void ObjectDetection::qsort_descent_inplace(std::vector<Object> &objects) {
  if (objects.empty())
    return;

  qsort_descent_inplace(objects, 0, objects.size() - 1);  // TODO(simon) Magic number.
}
void ObjectDetection::qsort_descent_inplace(std::vector<Object> &faceobjects,
                                            int left,
                                            int right) {  // TODO(simon) This gives error.
  int i = left;
  int j = right;
  float p = faceobjects[(left + right) / 2].prob;

  while (i <= j) {
    while (faceobjects[i].prob > p)
      i++;

    while (faceobjects[j].prob < p)
      j--;

    if (i <= j) {
      // swap
      std::swap(faceobjects[i], faceobjects[j]);

      i++;
      j--;
    }
  }

#pragma omp parallel sections
  {
#pragma omp section
    {
      if (left < j) qsort_descent_inplace(faceobjects, left, j);
    }
#pragma omp section
    {
      if (i < right) qsort_descent_inplace(faceobjects, i, right);
    }
  }
}
void ObjectDetection::nms_sorted_bboxes(const std::vector<Object> &faceobjects,
                                        std::vector<int> &picked,
                                        float nms_threshold) {
  picked.clear();

  const int n = faceobjects.size();

  std::vector<float> areas(n);
  for (int i = 0; i < n; i++) {  // TODO(simon) Magic number.
    areas[i] = faceobjects[i].rect.area();
  }

  for (int i = 0; i < n; i++) {  // TODO(simon) Magic number.
    const Object &a = faceobjects[i];

    int keep = 1;  // TODO(simon) Magic number.
    for (int j = 0; j < static_cast<int>(picked.size()); j++) {  // TODO(simon) Magic number.
      const Object &b = faceobjects[picked[j]];

      // intersection over union
      float inter_area = intersection_area(a, b);
      float union_area = areas[i] + areas[picked[j]] - inter_area;
      // float IoU = inter_area / union_area
      if (inter_area / union_area > nms_threshold)
        keep = 0;  // TODO(simon) Magic number.
    }

    if (keep)
      picked.emplace_back(i);
  }
}

float ObjectDetection::intersection_area(const Object &a, const Object &b) {
  cv::Rect_<float> inter = a.rect & b.rect;
  return inter.area();
}

void ObjectDetection::draw_objects(const cv::Mat &bgr, const std::vector<Object> &objects) {
  static const char
      *class_names[] = {  // TODO(simon) Class names should be set in the configuration.
      "pallet"
  };

  detection_output_struct_.clear();
  detection_output_struct_.resize(objects.size());

  for (size_t i = 0; i < objects.size(); i++) {   // TODO(simon) Magic number.
    const Object &obj = objects[i];

    cv::Scalar color = bounding_box_color_;
    float c_mean = cv::mean(color)[0];  // TODO(simon) Magic number.
    cv::Scalar txt_color;
    if (c_mean > 0.5) {  // TODO(simon) Magic number.
      txt_color = cv::Scalar(0, 0, 0);  // TODO(simon) Magic number.
    } else {
      txt_color = cv::Scalar(255, 255, 255);  // TODO(simon) Magic number.
    }

    cv::rectangle(bgr, obj.rect, color * 255, 2);  // TODO(simon) Magic number.

    char text[256];  // TODO(simon) Magic number.
    sprintf(text,  // TODO(simon) Never use sprintf. Use snprintf instead.
            "%s %.1f%%",
            class_names[obj.label],
            obj.prob * 100);  // TODO(simon) Magic number.

    int baseLine = 0;  // TODO(simon) Magic number.
    cv::Size label_size = cv::getTextSize(text,
                                          cv::FONT_HERSHEY_SIMPLEX,
                                          0.4,
                                          1,
                                          &baseLine);  // TODO(simon) Magic number.

    cv::Scalar txt_bk_color = color * 0.7 * 255;  // TODO(simon) Magic number.

    int x = obj.rect.x;
    int y = obj.rect.y + 1;  // TODO(simon) Magic number.
    if (y > bgr.rows)
      y = bgr.rows;

    cv::rectangle(bgr,
                  cv::Rect(cv::Point(x, y),
                           cv::Size(label_size.width, label_size.height + baseLine)),
                  txt_bk_color,
                  -1);  // TODO(simon) Magic number.
    // obj.rect.x, obj.rect.y, obj.rect.width, obj.rect.height
    detection_output_struct_.at(i).x = obj.rect.x;
    detection_output_struct_.at(i).y = obj.rect.y;
    detection_output_struct_.at(i).width = obj.rect.width;
    detection_output_struct_.at(i).height = obj.rect.height;
    detection_output_struct_.at(i).confidence = static_cast<double>(obj.prob);

    cv::putText(bgr, text, cv::Point(x, y + label_size.height),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, txt_color, 1);  // TODO(simon) Magic number.
  }
}

object_detection_output ObjectDetection::get_detection() {
  double max_confidence = 0;
  double max_areal = 0;
  double max_bbox_score = 0;  // TODO(simon) Magic number.
  int iterator_max_confidence = 0;  // TODO(simon) Magic number.
  uint32_t image_width =
      1280;  // TODO(simon) Set this as an input image_.cols  // TODO(simon) Magic number.
  uint32_t image_height =
      720;  // TODO(simon) Set this as an input image_.rows  // TODO(simon) Magic number.


  if (!detection_output_struct_.empty()) {  // TODO(simon) Implement pallet selection with enum pallet_selection_method from PoseEstimation.h
    for (int i = 0; i < detection_output_struct_.size(); ++i) {  // TODO(simon) Magic number.
//      if (detection_output_struct_.at(i).confidence > max_confidence){
//        max_confidence = detection_output_struct_.at(i).confidence; // TODO(simon) Select for max confidence
//        iterator_max_confidence = i;
//      }
//      if (detection_output_struct_.at(i).width * detection_output_struct_.at(i).height > max_areal){
//        max_areal = detection_output_struct_.at(i).width * detection_output_struct_.at(i).height; // TODO(simon) Select for largest size
//        iterator_max_confidence = i;
//      }
//      if (detection_output_struct_.at(i).y + detection_output_struct_.at(i).height > max_areal){
//        max_areal = detection_output_struct_.at(i).y + detection_output_struct_.at(i).height; // TODO(simon) Select for lowest detected position
//        iterator_max_confidence = i;
//      }
      if (box_filtering(image_width, image_height, detection_output_struct_, i) > max_bbox_score) {// TODO(simon) Select for lowest detected position and most center.
        max_bbox_score = box_filtering(image_width, image_height, detection_output_struct_, i);
        iterator_max_confidence = i;
      }
    }
  }

  if (!detection_output_struct_.empty()) {
    return detection_output_struct_.at(iterator_max_confidence);
  }
  object_detection_output non_detect{1, 1, 1, 1, 1.0};  // TODO(simon) Magic number.


  return non_detect;
}
void ObjectDetection::set_model_path(std::string path) {
  model_path_ = path;
}
void ObjectDetection::set_object_detection_settings(float nms_threshold,
                                                    float bbox_conf_threshold) {
  nms_threshold_ = nms_threshold;  // Default 0.45
  bbox_conf_threshold_ = bbox_conf_threshold;  // Default 0.25 or 0.75
}
double ObjectDetection::box_filtering(double image_width,  // TODO(simon) Filtering of selecting the middle most down box
                                      double image_height,
                                      std::vector<object_detection_output> detection,
                                      uint16_t i) {
  double height_fraction_score = 0;
  double width_fraction_score = 0;
  double width_fraction_bias = 0.2;  //! Added constant  // TODO(simon) Magic number.

  std::vector<double> bbox_center_bottom = {static_cast<double>(detection_output_struct_.at(i).x) +
      static_cast<double>(detection_output_struct_.at(i).width) / 2,
                                            static_cast<double>(detection_output_struct_.at(i).y) +
                                                static_cast<double>(detection_output_struct_.at(i).height)};

  height_fraction_score = bbox_center_bottom.at(1) / image_height;  // TODO(simon) Magic number.

  if (bbox_center_bottom.at(0)
      > image_width / 2) {  //! Right side of image  // TODO(simon) Magic number.
    width_fraction_score = 1 - ((bbox_center_bottom.at(0) - (image_width / 2))
        / (image_width / 2));  // TODO(simon) Magic number.
  } else {  //! Left side of image
    width_fraction_score =
        bbox_center_bottom.at(0) / (image_width / 2);  // TODO(simon) Magic number.
  }

  width_fraction_score = width_fraction_score + width_fraction_bias;

  return height_fraction_score + width_fraction_score;
}
