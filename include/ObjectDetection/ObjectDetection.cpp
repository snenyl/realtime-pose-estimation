//
// Created by nylund on 20.03.2022.
//

#include "ObjectDetection.h"

void ObjectDetection::setup_object_detection() {
//  model_path_ = "models/yolox_s_only_pallet_294epoch_o10/yolox_s_only_pallet_294epoch_o10.xml";
  input_model_path_ = std::filesystem::current_path().parent_path() / model_path_;

//  nms_threshold_ = 0.45; // Default 0.45
//  bbox_conf_threshold_ = 0.75; // Default 0.25
  num_classes_ = 1;
  input_dimensions_.width = 640;
  input_dimensions_.height = 640;
  device_name_ = "CPU";

  std::cout << input_model_path_ << std::endl;

  network_ = ie_.ReadNetwork(input_model_path_);

  if (network_.getOutputsInfo().size() != 1)
    std::cout << "Sample supports topologies with 1 output only" << std::endl;
  if (network_.getInputsInfo().size() != 1)
    std::cout << "Sample supports topologies with 1 input only" << std::endl;


  input_info_ = network_.getInputsInfo().begin()->second;
  input_name_ = network_.getInputsInfo().begin()->first;

  if (network_.getOutputsInfo().empty()) {
    std::cerr << "Network outputs info is empty" << std::endl;
//    return EXIT_FAILURE;
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
  if (!moutput_){
    std::cout << "We expect output to be inherited from MemoryBlob, "
                 "but by fact we were not able to cast output to MemoryBlob" << std::endl;
  }

  auto moutputHolder = moutput_->rmap();
  const auto* net_pred = moutputHolder.as<const InferenceEngine::PrecisionTrait<InferenceEngine::Precision::FP32>::value_type*>();

  int img_w = image.cols;
  int img_h = image.rows;

  float scale = std::min(input_dimensions_.width / (image.cols*1.0), input_dimensions_.height / (image.rows*1.0));

  decode_outputs(net_pred, objects_, scale, img_w, img_h);
  draw_objects(image,objects_);


}

cv::Mat ObjectDetection::static_resize(cv::Mat &img) {
  float r = std::min(input_dimensions_.width / (img.cols*1.0), input_dimensions_.height / (img.rows*1.0));
  // r = std::min(r, 1.0f);
  int unpad_w = r * img.cols;
  int unpad_h = r * img.rows;
  cv::Mat re(unpad_h, unpad_w, CV_8UC3);
  cv::resize(img, re, re.size());
  //cv::Mat out(INPUT_W, INPUT_H, CV_8UC3, cv::Scalar(114, 114, 114));
  cv::Mat out(input_dimensions_.height, input_dimensions_.width, CV_8UC3, cv::Scalar(114, 114, 114));
  re.copyTo(out(cv::Rect(0, 0, re.cols, re.rows)));
  return out;
}

void ObjectDetection::blobFromImage(cv::Mat &img, InferenceEngine::Blob::Ptr &blob) {
  int channels = 3;
  int img_h = img.rows;
  int img_w = img.cols;
  InferenceEngine::MemoryBlob::Ptr mblob = InferenceEngine::as<InferenceEngine::MemoryBlob>(blob);
  if (!mblob)
  {
    std::cout << "We expect blob to be inherited from MemoryBlob in matU8ToBlob, "
                       << "but by fact we were not able to cast inputBlob to MemoryBlob" << std::endl;
  }
  // locked memory holder should be alive all time while access to its buffer happens
  auto mblobHolder = mblob->wmap();

  float *blob_data = mblobHolder.as<float *>();

  for (size_t c = 0; c < channels; c++)
  {
    for (size_t  h = 0; h < img_h; h++)
    {
      for (size_t w = 0; w < img_w; w++)
      {
        blob_data[c * img_w * img_h + h * img_w + w] =
            (float)img.at<cv::Vec3b>(h, w)[c];
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
  std::vector<int> strides = {8, 16, 32};
  std::vector<GridAndStride> grid_strides;



  generate_grids_and_stride(input_dimensions_.width , input_dimensions_.height, strides, grid_strides);
  generate_yolox_proposals(grid_strides, prob,  bbox_conf_threshold_, proposals);

  if (proposals.size()>0){
    qsort_descent_inplace(proposals, 0, 0); // TODO(simon) Getting: "Process finished with exit code 139 (interrupted by signal 11: SIGSEGV)"
  }


  std::vector<int> picked;
  nms_sorted_bboxes(proposals, picked, nms_threshold_);
  int count = picked.size();
  objects.resize(count);

  for (int i = 0; i < count; i++)
  {
    objects[i] = proposals[picked[i]];

    // adjust offset to original unpadded
    float x0 = (objects[i].rect.x) / scale;
    float y0 = (objects[i].rect.y) / scale;
    float x1 = (objects[i].rect.x + objects[i].rect.width) / scale;
    float y1 = (objects[i].rect.y + objects[i].rect.height) / scale;

    // clip
    x0 = std::max(std::min(x0, (float)(img_w - 1)), 0.f);
    y0 = std::max(std::min(y0, (float)(img_h - 1)), 0.f);
    x1 = std::max(std::min(x1, (float)(img_w - 1)), 0.f);
    y1 = std::max(std::min(y1, (float)(img_h - 1)), 0.f);

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

  for (auto stride : strides)
  {
    int num_grid_w = target_w / stride;
    int num_grid_h = target_h / stride;
    for (int g1 = 0; g1 < num_grid_h; g1++)
    {
      for (int g0 = 0; g0 < num_grid_w; g0++)
      {
        grid_strides.push_back((GridAndStride){g0, g1, stride});
      }
    }
  }

}
void ObjectDetection::generate_yolox_proposals(std::vector<GridAndStride> grid_strides,
                                               const float *feat_ptr,
                                               float prob_threshold,
                                               std::vector<Object> &objects) {

  const int num_anchors = grid_strides.size();

  for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++)
  {
    const int grid0 = grid_strides[anchor_idx].grid0;
    const int grid1 = grid_strides[anchor_idx].grid1;
    const int stride = grid_strides[anchor_idx].stride;

    const int basic_pos = anchor_idx * (num_classes_ + 5);

    // yolox/models/yolo_head.py decode logic
    //  outputs[..., :2] = (outputs[..., :2] + grids) * strides
    //  outputs[..., 2:4] = torch.exp(outputs[..., 2:4]) * strides
    float x_center = (feat_ptr[basic_pos + 0] + grid0) * stride;
    float y_center = (feat_ptr[basic_pos + 1] + grid1) * stride;
    float w = exp(feat_ptr[basic_pos + 2]) * stride;
    float h = exp(feat_ptr[basic_pos + 3]) * stride;
    float x0 = x_center - w * 0.5f;
    float y0 = y_center - h * 0.5f;

    float box_objectness = feat_ptr[basic_pos + 4];
    for (int class_idx = 0; class_idx < num_classes_; class_idx++)
    {
      float box_cls_score = feat_ptr[basic_pos + 5 + class_idx];
      float box_prob = box_objectness * box_cls_score;
      if (box_prob > prob_threshold)
      {
        Object obj;
        obj.rect.x = x0;
        obj.rect.y = y0;
        obj.rect.width = w;
        obj.rect.height = h;
        obj.label = class_idx;
        obj.prob = box_prob;

        objects.emplace_back(obj);
      }

    } // class loop

  } // point anchor loop

}
void ObjectDetection::qsort_descent_inplace(std::vector<Object> &objects) {
  if (objects.empty())
    return;

  qsort_descent_inplace(objects, 0, objects.size() - 1);
}
void ObjectDetection::qsort_descent_inplace(std::vector<Object> &faceobjects, int left, int right) { // TODO(simon) This gives error.
  int i = left;
  int j = right;
  float p = faceobjects[(left + right) / 2].prob;

  while (i <= j)
  {
    while (faceobjects[i].prob > p)
      i++;

    while (faceobjects[j].prob < p)
      j--;

    if (i <= j)
    {
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
  for (int i = 0; i < n; i++)
  {
    areas[i] = faceobjects[i].rect.area();
  }

  for (int i = 0; i < n; i++)
  {
    const Object& a = faceobjects[i];

    int keep = 1;
    for (int j = 0; j < (int)picked.size(); j++)
    {
      const Object& b = faceobjects[picked[j]];

      // intersection over union
      float inter_area = intersection_area(a, b);
      float union_area = areas[i] + areas[picked[j]] - inter_area;
      // float IoU = inter_area / union_area
      if (inter_area / union_area > nms_threshold)
        keep = 0;
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

  static const char* class_names[] = {
      "pallet"
  };

  detection_output_struct_.clear();
  detection_output_struct_.resize(objects.size());

  for (size_t i = 0; i < objects.size(); i++)
  {
    const Object& obj = objects[i];


//    fprintf(stderr, "%d = %.5f at %.2f %.2f %.2f x %.2f\n", obj.label, obj.prob,
//            obj.rect.x, obj.rect.y, obj.rect.width, obj.rect.height);

    cv::Scalar color = cv::Scalar(100, 100,100);
    float c_mean = cv::mean(color)[0];
    cv::Scalar txt_color;
    if (c_mean > 0.5){
      txt_color = cv::Scalar(0, 0, 0);
    }else{
      txt_color = cv::Scalar(255, 255, 255);
    }

    cv::rectangle(bgr, obj.rect, color * 255, 2);

    char text[256];
    sprintf(text, "%s %.1f%%", class_names[obj.label], obj.prob * 100);

    int baseLine = 0;
    cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.4, 1, &baseLine);

    cv::Scalar txt_bk_color = color * 0.7 * 255;

    int x = obj.rect.x;
    int y = obj.rect.y + 1;
    //int y = obj.rect.y - label_size.height - baseLine;
    if (y > bgr.rows)
      y = bgr.rows;
    //if (x + label_size.width > image.cols)
    //x = image.cols - label_size.width;

    cv::rectangle(bgr, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                  txt_bk_color, -1);
    // obj.rect.x, obj.rect.y, obj.rect.width, obj.rect.height
    detection_output_struct_.at(i).x = obj.rect.x;
    detection_output_struct_.at(i).y = obj.rect.y;
    detection_output_struct_.at(i).width = obj.rect.width;
    detection_output_struct_.at(i).height = obj.rect.height;
    detection_output_struct_.at(i).confidence = static_cast<double>(obj.prob);

    cv::putText(bgr, text, cv::Point(x, y + label_size.height),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, txt_color, 1);
  }

}
object_detection_output ObjectDetection::get_detection() {

  double max_confidence = 0;
  double max_areal = 0;
  double max_bbox_score = 0;
  int iterator_max_confidence = 0;
  uint32_t image_width = 1280; // TODO(simon) Set this as an input image_.cols
  uint32_t image_height = 720; // TODO(simon) Set this as an input image_.rows


  if (!detection_output_struct_.empty()){
    for (int i = 0; i < detection_output_struct_.size(); ++i) {
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
      if (box_filtering(image_width,image_height,detection_output_struct_,i)>max_bbox_score){
        max_bbox_score = box_filtering(image_width,image_height,detection_output_struct_,i);
        iterator_max_confidence = i;
      }
    }
  }

  if (!detection_output_struct_.empty()) {
    return detection_output_struct_.at(iterator_max_confidence);
  }
  object_detection_output non_detect{1,1,1,1,1.0};


  return non_detect;
}
void ObjectDetection::set_model_path(std::string path) {
  model_path_ = path;
}
void ObjectDetection::set_object_detection_settings(float nms_threshold,
                                                    float bbox_conf_threshold) {
  nms_threshold_ = nms_threshold; // Default 0.45
  bbox_conf_threshold_ = bbox_conf_threshold; // Default 0.25 or 0.75
}
double ObjectDetection::box_filtering(double image_width, //TODO(simon) Filtering of selecting the middle most down box
                                      double image_height,
                                      std::vector<object_detection_output> detection,
                                      uint16_t i) {
  double height_fraction_score = 0;
  double width_fraction_score = 0;

  std::vector<double> bbox_center_bottom = {static_cast<double>(detection_output_struct_.at(i).x)+
                                            static_cast<double>(detection_output_struct_.at(i).width)/2,
                                            static_cast<double>(detection_output_struct_.at(i).y)+
                                            static_cast<double>(detection_output_struct_.at(i).height)};

  height_fraction_score = bbox_center_bottom.at(1)/image_height;

  if (bbox_center_bottom.at(0)>image_width/2){//! Right side of image
    width_fraction_score = 1-((bbox_center_bottom.at(0)-(image_width/2))/(image_width/2));
  }
  else{//! Left side of image
    width_fraction_score = bbox_center_bottom.at(0)/(image_width/2);
  }

  return height_fraction_score*width_fraction_score;
}
