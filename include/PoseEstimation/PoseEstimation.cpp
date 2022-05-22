//
// Created by nylund on 19.03.2022.
//

#include "PoseEstimation.h"

void PoseEstimation::run_pose_estimation() {
  rs2::frameset frames = p.wait_for_frames();
  rs2::video_frame image = frames.get_color_frame();
  rs2::depth_frame depth = frames.get_depth_frame();

  realsense_points_ = realsense_pointcloud_.calculate(depth);
  pcl_points_ = points_to_pcl(realsense_points_);

  detection_output_struct_ = object_detection_object_.get_detection();


  if (std::chrono::system_clock::now() > start_debug_time_){
    std::cout << " X: " << detection_output_struct_.x
              << " Y: " << detection_output_struct_.y
              << " Width: " << detection_output_struct_.width
              << " Height: " << detection_output_struct_.height
              << " Conf: " << detection_output_struct_.confidence << std::endl;
  }

  calculate_3d_crop();

  edit_pointcloud();

  if (std::chrono::system_clock::now() > start_debug_time_){
    std::cout << "cloud_pallet_->size(): " << cloud_pallet_->size() << std::endl;
  }

//  calculate_ransac(); //Debugging


  if (detection_output_struct_.width > 10  &&
      detection_output_struct_.height > 10 &&
      wait_with_ransac_for_ > 10){
    calculate_ransac();
  }
  if (ransac_model_coefficients_.size() > 3){
    calculate_pose_vector();
  }
  wait_with_ransac_for_ += 1;

  view_pointcloud();


  const int w = image.as<rs2::video_frame>().get_width();
  const int h = image.as<rs2::video_frame>().get_height();

  cv::Mat cv_image(cv::Size(w, h), CV_8UC3, (void*)image.get_data(), cv::Mat::AUTO_STEP);
  cv::cvtColor(cv_image,cv_image,cv::COLOR_BGR2RGB);

  image_ = cv_image;

  calculate_aruco();
  object_detection_object_.run_object_detection(image_);
  calculate_pose();


  log_data(image.get_frame_number());
  cv::imshow("Output",cv_image);
  cv::waitKey(1);

  ransac_model_coefficients_.clear();

}
void PoseEstimation::setup_pose_estimation() {
//  std::string config_path = std::filesystem::current_path().parent_path() / "config/realtime_pose_estimation_config.json";
//  std::ofstream config_file;
//  config_file.open(config_path);
//
//  while (config_file.is_open()){
//
//    std::cout << config_file.rdbuf() << std::endl;
//
//    config_file.close();
//  }
//
//  std::cout << "config_path: " << config_path << std::endl;


//  rosbag_path_ = std::filesystem::current_path().parent_path() / "data/ros_bags_27032022/20220327_161534_2meter_with_light_standing_aruco_0.bag";
  rosbag_path_ = std::filesystem::current_path().parent_path() / "data/ros_bags_27032022/20220327_162128_2meter_with_light_standing_aruco_90_deg_slow_move.bag";



  if (load_from_rosbag){
    std::cout << "Loaded rosbag: " << rosbag_path_ << std::endl;
//    p.stop(); // TODO(simon) check if stream is running. p.get_active_profile().get_device()
    rs2::config cfg;
    cfg.enable_device_from_file(rosbag_path_,!single_run_);
    auto profile = p.start(cfg);
    auto dev = profile.get_device();

    if (auto p = dev.as<rs2::playback>()){
      p.set_real_time(0); //! Doesn't skip frames. :)
    }

  }
  else if (!load_from_rosbag){
    p.start();
  }

  if (enable_logger_){
    std::ofstream LoggerFile(std::filesystem::current_path().parent_path() / "log/data_out.csv");
    LoggerFile << "frame,p_x,p_y,p_z,p_r,p_p,p_y,a_x,a_y,a_z,a_r,a_p,a_y" << std::endl;
    LoggerFile.close();

  }

//  rs2_get_video_stream_intrinsics()

  set_camera_parameters();

//  object_detection_object_.set_model_path("models/yolox_s_only_pallet_void_300epoch_o10/yolox_s_only_pallet_void_300epoch_lim_o10.xml");
  object_detection_object_.set_model_path("models/yolox_s_only_pallet_294epoch_o10/yolox_s_only_pallet_294epoch_o10.xml");
  object_detection_object_.set_object_detection_settings(0.3,0.1); //! Får med hele sekvensen: 0.3,0.1 | 0.3,0.75
  object_detection_object_.setup_object_detection();
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_ = viewer;

//  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9);

  std::cout << "Setup" << std::endl;

}
void PoseEstimation::calculate_aruco() {

  cv::aruco::detectMarkers(image_, dictionary_, markerCorners_, markerIds_, parameters_, rejectedCandidates_);

//  cv::aruco::CornerRefineMethod::CORNER_REFINE_APRILTAG


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

  if (!rvecs.empty() && !tvecs.empty()){
    double z_axis_data[3] = {0,0,1};
    cv::Mat Rot(3,3,CV_64F),Jacob;
    cv::Mat z_axis(1,3,CV_64F,z_axis_data);
    cv::Rodrigues(rvecs, Rot, Jacob);


    std::cout << "z_axis: " << z_axis << std::endl;
    std::cout << "Rot: " << Rot << std::endl;

    ground_truth_vector_ = z_axis*Rot;
  }


//  std::cout << rvecs.size() << "\n" << tvecs.size() << "\n" << object_points.size() << std::endl;
//  std::cout << rvecs.at(0) << "\n" << tvecs.at(0) << std::endl;


  if (!rvecs.empty() && !tvecs.empty()){
    std::stringstream rotation;
    std::stringstream translation;

    rvecs_ = rvecs;
    tvecs_ = tvecs;

//    rotation << rvecs.at(0);
    rotation << "[" << rvecs.at(0)[0] << ", " << rvecs.at(0)[1] << ", " << rvecs.at(0)[2] << "]";
    translation << tvecs.at(0);
//    std::cout << rvecs.size() << "\n" << tvecs.size() << "\n" << object_points.size() << std::endl;
//    std::cout << rvecs.at(0) << "\n" << tvecs.at(0) << std::endl;
    cv::putText(image_,rotation.str(),cv::Point(50,50),cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,255,0),2,false);
    cv::putText(image_,translation.str(),cv::Point(50,100),cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,255,0),2,false);
    cv::aruco::drawAxis(image_,example_camera_matrix_,example_dist_coefficients_,rvecs,tvecs,0.535/2);
  }

//  cv::aruco::drawAxis(image_,example_camera_matrix,example_dist_coefficients,rvecs,tvecs,0.1);

}

void PoseEstimation::set_camera_parameters() { // TODO(simon) Not in use.
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

pcl::PointCloud<pcl::PointXYZ>::Ptr PoseEstimation::points_to_pcl(const rs2::points &points) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  auto sp = points.get_profile().as<rs2::video_stream_profile>();
  cloud->width = sp.width();
  cloud->height = sp.height();
  cloud->is_dense = false;
  cloud->points.resize(points.size());
  auto ptr = points.get_vertices();
  for (auto& p : cloud->points)
  {
    p.x = ptr->x;
    p.y = ptr->y;
    p.z = ptr->z;
    ptr++;
  }

  return cloud;
}
void PoseEstimation::edit_pointcloud() {

  pcl::FrustumCulling<pcl::PointXYZ> frustum_filter;

  pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_pallet(new pcl::PointCloud<pcl::PointXYZ>);

  local_cloud = pcl_points_;
  frustum_filter_inliers_.clear();

//  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud (new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Matrix4f camera_pose;
  Eigen::Matrix4f rotation_red_pos_ccw;
  Eigen::Matrix4f rotation_green_pos_cw;

  camera_pose <<  0, 0, -1, 0,
                  0, 1,  0, 0,
                  1, 0,  0, 0,
                  0, 0,  0, 1;


  Eigen::Vector2d z_inverse;
  z_inverse << 1,0;
  Eigen::Vector2d center_frustum_zy;
  Eigen::Vector2d center_frustum_zx;
  center_frustum_zy << center_frustum_.z,center_frustum_.y;
  center_frustum_zx << center_frustum_.z,center_frustum_.x;
  double angle_zy = std::acos((z_inverse.dot(center_frustum_zy))/(z_inverse.norm()*center_frustum_zy.norm()));
  double angle_zx = std::acos((z_inverse.dot(center_frustum_zx))/(z_inverse.norm()*center_frustum_zx.norm()));


  if (std::chrono::system_clock::now() > start_debug_time_){
    std::cout << "\n ROTATION: \n " <<  std::endl;
    std::cout << "angle_zy: " << angle_zy << std::endl;
    std::cout << "angle_zx: " << angle_zx << std::endl;
    std::cout << "center_frustum_zy: " << center_frustum_zy.x() << " " << center_frustum_zy.y() << std::endl;
    std::cout << "center_frustum_zx: " << center_frustum_zx.x() << " " << center_frustum_zx.y() << std::endl;

    std::cout << center_frustum_ << std::endl;

    std::cout << "z_inverse.dot(center_frustum_zy): " << z_inverse.dot(center_frustum_zy) << std::endl;
    std::cout << "z_inverse: " << z_inverse << std::endl;
    std::cout << "center_frustum_zy: " << center_frustum_zy << std::endl;
    std::cout << "angle_zy: " << angle_zy << std::endl;
    std::cout << "angle_zx: " << angle_zx << std::endl;

  }

  if (center_frustum_zx.y()>0){ // TODO(simon) This is a hack. Solve the problem not the symptom.
    angle_zx *= -1;
  }
  if (center_frustum_zy.y()<0){ // TODO(simon) This is a hack. Solve the problem not the symptom.
    angle_zy *= -1;
  }

  float rot_red_rad = angle_zy;
  float rot_green_rad = angle_zx;

  rotation_red_pos_ccw << std::cos(rot_red_rad),-std::sin(rot_red_rad), 0,0,
                          std::sin(rot_red_rad), std::cos(rot_red_rad), 0,0,
                          0, 0, 1,  0,
                          0, 0,  0, 1;

  rotation_green_pos_cw << std::cos(rot_green_rad), 0,-std::sin(rot_green_rad), 0,
                          0,                         1,                          0, 0,
                          std::sin(rot_green_rad), 0, std::cos(rot_green_rad), 0,
                          0,                         0,                          0, 1;

  frustum_filter.setInputCloud(local_cloud);
  frustum_filter.setCameraPose(camera_pose*rotation_red_pos_ccw*rotation_green_pos_cw); //   frustum_filter.setCameraPose(camera_pose*cam2robot*rotation_red_pos_ccw*rotation_green_pos_cw);
  frustum_filter.setNearPlaneDistance(0);
  frustum_filter.setFarPlaneDistance(15);
  frustum_filter.setVerticalFOV(fov_v_rad_ * 57.2958);
  frustum_filter.setHorizontalFOV(fov_h_rad_ * 57.2958);
  frustum_filter.filter(*local_pallet);
  frustum_filter.filter(frustum_filter_inliers_);


  cloud_pallet_ = local_pallet;

  if (std::chrono::system_clock::now() > start_debug_time_){
    std::cout << "local_pallet->size() " <<  local_pallet->size() << std::endl;
    std::cout << "cloud_pallet_->size() " <<  cloud_pallet_->size() << std::endl;
    std::cout << "local_cloud->size() " <<  local_cloud->size() << std::endl;
  }




}
void PoseEstimation::view_pointcloud() {

    if (first_run_){
    viewer_->setBackgroundColor (0, 0, 0);
    viewer_->addCoordinateSystem (1);
    viewer_->initCameraParameters();
    viewer_->setCameraPosition(0, 10, 0,    0, 0, 0,   0, 0, 1);
    first_run_ = false;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud_view(new pcl::PointCloud<pcl::PointXYZRGB>);

  final_cloud_view->clear();
  viewer_->removeAllShapes();
  viewer_->removeAllPointClouds();
  viewer_->removeCoordinateSystem("aruco_marker",0);

  pcl::copyPointCloud(*pcl_points_,*final_cloud_view);

  for (int i = 0; i < final_cloud_view->points.size(); ++i) {
    final_cloud_view->points[i].r = 255;
    final_cloud_view->points[i].g = 255;
    final_cloud_view->points[i].b = 255;
  }

  if (frustum_filter_inliers_.size()>10){
//    std::cout << frustum_filter_inliers_.size() << std::endl;
    for (int i = 0; i < frustum_filter_inliers_.size(); ++i) {
      final_cloud_view->points[frustum_filter_inliers_.at(i)].b=0;
    }
  }

//  float vector_length = 0.1; // TODO(simon) Input as parameter.

//  if (output_cloud_with_normals_->size() > 10){ //! This is slow, use only for debugging.
//    for (int i = 0; i < output_cloud_with_normals_->size(); ++i) {
//      viewer_->addLine(pcl::PointXYZ(output_cloud_with_normals_->at(i).x,
//                                     output_cloud_with_normals_->at(i).y,
//                                     output_cloud_with_normals_->at(i).z),
//                       pcl::PointXYZ(output_cloud_with_normals_->at(i).x+output_cloud_with_normals_->at(i).normal_x*vector_length,
//                                     output_cloud_with_normals_->at(i).y+output_cloud_with_normals_->at(i).normal_y*vector_length,
//                                     output_cloud_with_normals_->at(i).z+output_cloud_with_normals_->at(i).normal_z*vector_length),
//                       std::to_string(i),0);
//    }
//  }


//  for (int i = 0; i < inliers_->indices.size(); ++i) {
//    final_cloud_view->points[inliers_->indices.at(i)].g = 0;
//    final_cloud_view->points[inliers_->indices.at(i)].b = 0;
//  }

//! Adding and removing Pointclouds
  viewer_->addPointCloud(final_cloud_view,"final_cloud",0); //! Everything with color
//  viewer_->addPointCloud(cloud_pallet_); //! Only pallet
//  viewer_->addPointCloud(final_); //! RANSAC test
//  viewer_->addPointCloudNormals<pcl::PointNormal>(output_cloud_with_normals_,100,0.1f,"final_normals",0);
//  viewer_->addPointCloudNormals<pcl::PointNormal>(extracted_cloud_with_normals_,100,0.1f,"extracted_normals",0);

//  pcl::io::savePCDFileASCII("/home/nylund/Documents/git_uia/realtime_pose_estimation/exports/cloud_pallet_.pcd",*cloud_pallet_);
//  pcl::io::savePCDFileASCII("/home/nylund/Documents/git_uia/realtime_pose_estimation/exports/final_cloud_view.pcd",*final_cloud_view);



  if (!rvecs_.empty() && !tvecs_.empty()){
    Eigen::Affine3f rotation;
//    std::cout << "RVECS: " << rvecs_.at(0) << std::endl;
//    std::cout << "TVECS: " << tvecs_.at(0) << std::endl;
//    viewer_->addCoordinateSystem(0.535,0.128196, 0.0394752, 0.815717,"aruco_marker",0);

    pcl::PointXYZ startpoint = pcl::PointXYZ(tvecs_.at(0)[0],tvecs_.at(0)[1],tvecs_.at(0)[2]);
//    pcl::PointXYZ startpoint = pcl::PointXYZ(0,0,0);
//    pcl::PointXYZ endpoint = pcl::PointXYZ(tvecs_.at(0)[0]+rvecs_.at(0)[0],tvecs_.at(0)[1]+rvecs_.at(0)[1],tvecs_.at(0)[2]+rvecs_.at(0)[2]);
    std::vector<double>ground_truth_vector_converted(ground_truth_vector_.begin<double>(), ground_truth_vector_.end<double>());

    pcl::PointXYZ endpoint = pcl::PointXYZ(tvecs_.at(0)[0]-ground_truth_vector_converted.at(0), //TODO(simon) Testing remove "*2" Justering er ikke linjær
                                           tvecs_.at(0)[1]+ground_truth_vector_converted.at(1),
                                           tvecs_.at(0)[2]-ground_truth_vector_converted.at(2)); // TODO(simon) Testing remove "-1.75"


    converted_ground_truth_vector_.at(0) = tvecs_.at(0)[0];
    converted_ground_truth_vector_.at(1) = tvecs_.at(0)[1];
    converted_ground_truth_vector_.at(2) = tvecs_.at(0)[2];
    converted_ground_truth_vector_.at(3) = -ground_truth_vector_converted.at(0);
    converted_ground_truth_vector_.at(4) = ground_truth_vector_converted.at(1);
    converted_ground_truth_vector_.at(5) = -ground_truth_vector_converted.at(2);

    for (int i = 0; i < converted_ground_truth_vector_.size(); ++i) {
      std::cout << "converted_ground_truth_vector_.at("<<i<<")" << converted_ground_truth_vector_.at(i) << std::endl;
    }

    std::cout << "ENDPOINT: " << endpoint << std::endl;

    viewer_->addLine(startpoint,
                     endpoint,0,0,255,
                     "ground_truth",0);


  }

  if (std::chrono::system_clock::now() > start_debug_time_){
    std::cout << "square_frustum_detection_points_.at(0)" << square_frustum_detection_points_.at(0) << std::endl;
    std::cout << "square_frustum_detection_points_.at(1)" << square_frustum_detection_points_.at(1) << std::endl;
    std::cout << "square_frustum_detection_points_.at(2)" << square_frustum_detection_points_.at(2) << std::endl;
    std::cout << "square_frustum_detection_points_.at(3)" << square_frustum_detection_points_.at(3) << std::endl;
    start_debug_time_ = std::chrono::system_clock::now();
    start_debug_time_ += std::chrono::seconds(5);
  }

  if (!square_frustum_detection_points_.empty()){
    viewer_->addLine(pcl::PointXYZ(0,0,0),
                     square_frustum_detection_points_.at(0),255,255,0,
                     "line",0);
    viewer_->addLine(pcl::PointXYZ(0,0,0),
                     square_frustum_detection_points_.at(1),255,255,0,
                     "line1",0);
    viewer_->addLine(pcl::PointXYZ(0,0,0),
                     square_frustum_detection_points_.at(2),255,255,0,
                     "line2",0);
    viewer_->addLine(pcl::PointXYZ(0,0,0),
                     square_frustum_detection_points_.at(3),255,255,0,
                     "line3",0);
    viewer_->addLine(pcl::PointXYZ(0,0,0),
                     center_frustum_,255,0,0,
                     "line_center",0);
  }

  if (ransac_model_coefficients_.size() > 2){
    pcl::ModelCoefficients coff;
    coff.values = ransac_model_coefficients_;
    viewer_->addPlane(coff,0.0,0.0,0.0,"plane",0);
  }

  if (second_ransac_model_coefficients_.size() > 2){
    pcl::ModelCoefficients coff;
    coff.values = second_ransac_model_coefficients_;
    viewer_->addPlane(coff,0.0,0.0,0.0,"pallet_plane",0);
  }


//  if (first_ransac_model_coefficients_.size() > 2){
//    pcl::ModelCoefficients coff;
//    coff.values = first_ransac_model_coefficients_;
//    viewer_->addPlane(coff,0.0,0.0,0.0,"first_plane",0);
//  }




//  if (intersect_point_.values.size() > 3){
//    viewer_->addSphere(intersect_point_,"circle",0);
//  }


  if (ransac_model_coefficients_.size() > 2){
    viewer_->addLine(plane_frustum_vector_intersect_,
                     pose_vector_end_point_,0,255,0,
                     "pose_vector",0);
  }






//    while (LoggerFile.is_open()){
//          LoggerFile << plane_frustum_vector_intersect_.x << ","
//                     << plane_frustum_vector_intersect_.y << ","
//                     << plane_frustum_vector_intersect_.z << ","
////                     << ransac_model_coefficients_.at(0) << ","
////                     << ransac_model_coefficients_.at(1) << ","
////                     << ransac_model_coefficients_.at(2) << ","
////                     << tvecs_.at(0) << ","
////                     << tvecs_.at(1) << ","
////                     << tvecs_.at(2) << ","
////                     << rvecs_.at(0) << ","
////                     << rvecs_.at(1) << ","
////                     << rvecs_.at(2)
//                     << std::endl;
//          LoggerFile.close();
//    }
//  }




  viewer_->spinOnce(1);

}
void PoseEstimation::calculate_3d_crop() {

  std::vector<Eigen::Vector2d> detection_point_vec(4);

  Eigen::Vector2d image_center(zed_k_matrix_[2], //TODO(simon) Get K matrix from intel realsense
                               zed_k_matrix_[3]); //TODO(simon) Get K matrix from intel realsense

  detection_point_vec.at(0).x() = detection_output_struct_.x;
  detection_point_vec.at(0).y() = detection_output_struct_.y;

  detection_point_vec.at(1).x() = detection_output_struct_.x + detection_output_struct_.width;
  detection_point_vec.at(1).y() = detection_output_struct_.y;

  detection_point_vec.at(2).x() = detection_output_struct_.x;
  detection_point_vec.at(2).y() = detection_output_struct_.y + detection_output_struct_.height;

  detection_point_vec.at(3).x() = detection_output_struct_.x + detection_output_struct_.width;
  detection_point_vec.at(3).y() = detection_output_struct_.y + detection_output_struct_.height;

  detection_from_image_center_.clear();
  square_frustum_detection_points_.clear();
  for (int i = 0; i < 4; ++i) {
    detection_from_image_center_.emplace_back((detection_point_vec.at(i) - image_center) / zed_k_matrix_[0]); //TODO(simon) Get K matrix from intel realsense; The difference bwetween this is too spall
//    detection_from_image_center_.at(i).y() *= -1; // TODO(simon) This is a hack to invert the y-axis.
    square_frustum_detection_points_.emplace_back(detection_from_image_center_.at(i).x()*detection_vector_scale_,
                                                  detection_from_image_center_.at(i).y()*detection_vector_scale_,
                                                  detection_vector_scale_);
  }

  if (std::chrono::system_clock::now() > start_debug_time_){
    for (int i = 0; i < 4; ++i) {
      std::cout << "image_center X: " << image_center << std::endl;
      std::cout << "detection_point_vec.at(i): " << detection_point_vec.at(i) << std::endl;
      std::cout << "zed_k_matrix_[0] " << zed_k_matrix_[0] << std::endl;
      std::cout << "detection_from_image_center_: " << detection_from_image_center_.at(i) << std::endl;
    }
  }


  Eigen::Vector3f vector_0(square_frustum_detection_points_.at(0).x,
                           square_frustum_detection_points_.at(0).y,
                           square_frustum_detection_points_.at(0).z);

  Eigen::Vector3f vector_1(square_frustum_detection_points_.at(1).x,
                           square_frustum_detection_points_.at(1).y,
                           square_frustum_detection_points_.at(1).z);

  Eigen::Vector3f vector_2(square_frustum_detection_points_.at(2).x,
                           square_frustum_detection_points_.at(2).y,
                           square_frustum_detection_points_.at(2).z);

  Eigen::Vector3f vector_3(square_frustum_detection_points_.at(3).x,
                           square_frustum_detection_points_.at(3).y,
                           square_frustum_detection_points_.at(3).z);

  for (int i = 0; i < 4; ++i) {
    center_frustum_.x += square_frustum_detection_points_.at(i).x;
    center_frustum_.y += square_frustum_detection_points_.at(i).y;
    center_frustum_.z += square_frustum_detection_points_.at(i).z;
  }
  center_frustum_.x /= 4;
  center_frustum_.y /= 4;
  center_frustum_.z /= 4;

  Eigen::Vector3f vector_center(center_frustum_.x,
                                center_frustum_.y,
                                center_frustum_.z);


  Eigen::Vector3f vector_camera_front(1,0,0);



  float dot_01 = vector_0.dot(vector_1);
  float dot_02 = vector_0.dot(vector_2);

  float len_sq_0 = vector_0.norm();
  float len_sq_1 = vector_1.norm();
  float len_sq_2 = vector_2.norm();

  float angle_01 = std::acos(dot_01/(len_sq_0*len_sq_1));
  float angle_02 = std::acos(dot_02/(len_sq_0*len_sq_2));

//  std::cout << "dot: " << dot << " len_sq_0: " << len_sq_0 << " len_sq_1: " << len_sq_1 << " Angle: " << angle << std::endl;
//  std::cout << "angle_01: " << angle_01*57.2958 << " angle_02: " << angle_02*57.2958 << std::endl;

  fov_h_rad_ = angle_01;
  fov_v_rad_ = angle_02;

//  std::cout << "Vector 0: " << vector_0 << std::endl;
//  std::cout << "Vector 1: " << vector_1 << std::endl;
//  std::cout << "Vector 2: " << vector_2 << std::endl;
//  std::cout << "Vector 3: " << vector_3 << std::endl;
//  std::cout << "vector_center: " << vector_center << std::endl;

  if (std::chrono::system_clock::now() > start_debug_time_){
    std::cout << "detection_point_vec_0: " << detection_point_vec.at(0).x() << " " << detection_point_vec.at(0).y() << std::endl;
    std::cout << "detection_point_vec_1: " << detection_point_vec.at(1).x() << " " << detection_point_vec.at(1).y() << std::endl;
    std::cout << "detection_point_vec_2: " << detection_point_vec.at(2).x() << " " << detection_point_vec.at(2).y() << std::endl;
    std::cout << "detection_point_vec_3: " << detection_point_vec.at(3).x() << " " << detection_point_vec.at(3).y() << std::endl;
  }



//  cv::waitKey(0);

}
void PoseEstimation::calculate_ransac() {

  pcl::ModelCoefficients::Ptr first_coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr first_inliers (new pcl::PointIndices);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);

  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setEpsAngle(0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations(50);
  seg.setDistanceThreshold (0.001);

  std::cout << "First RANSAC" << std::endl;
  std::cout << "cloud_pallet_ size: " << cloud_pallet_->size() << std::endl;

  if (cloud_pallet_->size()>10){ //TODO(simon) 10 should be set as input parameter.
    seg.setInputCloud (cloud_pallet_);
    seg.segment (*first_inliers, *first_coefficients);


    first_ransac_model_coefficients_.clear();
    for (int i = 0; i < first_coefficients->values.size(); ++i) {
      first_ransac_model_coefficients_.emplace_back(first_coefficients->values.at(i));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> test_inliers;
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
        model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud_pallet_));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (0.0001);
    ransac.computeModel();
    ransac.getInliers(test_inliers);

    pcl::copyPointCloud (*cloud_pallet_, test_inliers, *final);
    final_ = final;
  }


  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

//  if (inliers_->indices.size()>10){
//    inliers_->indices.clear();
//  }

  pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr output_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr final_with_normals(new pcl::PointCloud<pcl::PointNormal>);

  input_cloud_with_normals->clear();
  output_cloud_with_normals->clear();
  final_with_normals->clear();

  input_cloud_with_normals->resize(cloud_pallet_->size());
  for (int i = 0; i < cloud_pallet_->points.size(); ++i) {
    input_cloud_with_normals->points.at(i).x = cloud_pallet_->at(i).x;
    input_cloud_with_normals->points.at(i).y = cloud_pallet_->at(i).y;
    input_cloud_with_normals->points.at(i).z = cloud_pallet_->at(i).z;
  }

  std::cout << "Sampling surface normals" << std::endl;
  std::cout << "input_cloud_with_normals size: " << input_cloud_with_normals->size() << std::endl;

//  //Sampling surface normals
  if (input_cloud_with_normals->size()>10){ //TODO(simon) 10 should be set as input parameter.
    pcl::SamplingSurfaceNormal<pcl::PointNormal> sample_surface_normal;
    sample_surface_normal.setInputCloud(input_cloud_with_normals);
    sample_surface_normal.setSample(50); // TODO(simon) Setting that is required to be a parameter.
    sample_surface_normal.setRatio(0.5); // TODO(simon) Setting that is required to be a parameter.
    sample_surface_normal.filter(*output_cloud_with_normals);

    output_cloud_with_normals_ = output_cloud_with_normals;
    final_with_normals_ = final_with_normals;

    std::vector<int> temp_index;
    pcl::removeNaNNormalsFromPointCloud(*output_cloud_with_normals_,*output_cloud_with_normals_,temp_index);

    int counter = 0;

    for (int i = 0; i < output_cloud_with_normals_->size(); ++i) {
      if (abs(output_cloud_with_normals_->at(i).normal_y)<0.45 && //! Default 0.45 or 0.10
          abs(output_cloud_with_normals_->at(i).x)>0.2 &&  //! Remove vector at origo.
          abs(output_cloud_with_normals_->at(i).y)>0.2 && //! Remove vector at origo.
          abs(output_cloud_with_normals_->at(i).z)>0.2) //! Remove vector at origo.
      {
        final_with_normals_->emplace_back(output_cloud_with_normals_->at(i));
        counter++;
      }
    }
  }


  // RANSAC
  if (input_cloud_with_normals->size()>10){ //TODO(simon) 10 should be set as input parameter.
    pcl::SACSegmentationFromNormals<pcl::PointNormal,pcl::PointNormal> segmentation;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_NORMAL_PLANE); // TODO(simon) Test with different models SACMODEL_PLANE | SACMODEL_NORMAL_PLANE | SACMODEL_PERPENDICULAR_PLANE
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(500);
    segmentation.setDistanceThreshold(0.1);

    std::cout << "Inbetween " << std::endl;
    std::cout << "input_cloud_with_normals size: " << input_cloud_with_normals->size() << std::endl;

    segmentation.setEpsAngle(0.1);
    segmentation.setInputCloud(output_cloud_with_normals_);
    segmentation.setInputNormals(output_cloud_with_normals_);

    segmentation.segment (*inliers, *coefficients);

    ransac_model_coefficients_.clear();
    for (int i = 0; i < coefficients->values.size(); ++i) {
      ransac_model_coefficients_.emplace_back(coefficients->values.at(i));
    }
  }


  // Extract filter

  if (output_cloud_with_normals_->size()>10){ //TODO(simon) 10 should be set as input parameter.
    pcl::PointCloud<pcl::PointNormal>::Ptr extracted_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    //Extract all points
    pcl::ExtractIndices<pcl::PointNormal> extract_filter;
    extract_filter.setInputCloud(output_cloud_with_normals_);
    extract_filter.setNegative(true);
    extract_filter.setIndices(inliers);
    extract_filter.filter(*extracted_cloud_with_normals);

    extracted_cloud_with_normals_ = extracted_cloud_with_normals;
  }

  // RANSAC 2

  if (extracted_cloud_with_normals_->size()>10){ //TODO(simon) 10 should be set as input parameter.
    pcl::SACSegmentationFromNormals<pcl::PointNormal,pcl::PointNormal> second_segmentation;
    pcl::ModelCoefficients::Ptr second_coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr second_inliers (new pcl::PointIndices);

    second_segmentation.setOptimizeCoefficients(true);
    second_segmentation.setModelType(pcl::SACMODEL_NORMAL_PLANE); // TODO(simon) Test with different models SACMODEL_PLANE | SACMODEL_NORMAL_PLANE | SACMODEL_PERPENDICULAR_PLANE
    second_segmentation.setMethodType(pcl::SAC_RANSAC);
    second_segmentation.setMaxIterations(500);
    second_segmentation.setDistanceThreshold(0.1);

    std::cout << "second_segmentation " << std::endl;
    std::cout << "extracted_cloud_with_normals_ size: " << extracted_cloud_with_normals_->size() << std::endl;

    second_segmentation.setEpsAngle(0.1);
    second_segmentation.setInputCloud(extracted_cloud_with_normals_);
    second_segmentation.setInputNormals(extracted_cloud_with_normals_);

    second_segmentation.segment (*second_inliers, *second_coefficients);

    second_ransac_model_coefficients_.clear();
    for (int i = 0; i < second_coefficients->values.size(); ++i) {
      second_ransac_model_coefficients_.emplace_back(second_coefficients->values.at(i));
    }
  }

  inliers_ = inliers;
}


Eigen::Affine3f PoseEstimation::create_rotation_matrix(float ax, float ay, float az) {
  Eigen::Affine3f rx =
      Eigen::Affine3f(Eigen::AngleAxisf(ax, Eigen::Vector3f(1, 0, 0)));
  Eigen::Affine3f ry =
      Eigen::Affine3f(Eigen::AngleAxisf(ay, Eigen::Vector3f(0, 1, 0)));
  Eigen::Affine3f rz =
      Eigen::Affine3f(Eigen::AngleAxisf(az, Eigen::Vector3f(0, 0, 1)));
  return rz * ry * rx;
}

void PoseEstimation::calculate_pose_vector() {
  // Calculating plane, vector intersection point and

  intersect_point_.values.resize(4);
  float distance_scalar = 0;
  Eigen::Vector3f center_frustum_vector;
  Eigen::Vector3f plane_vector_intersect;
  Eigen::Vector3f plane_orgin;
  Eigen::Vector3f plane_normal_vector;

;
  plane_vector_intersect_.x = -ransac_model_coefficients_.at(0)*ransac_model_coefficients_.at(3);
  plane_vector_intersect_.y = -ransac_model_coefficients_.at(1)*ransac_model_coefficients_.at(3);
  plane_vector_intersect_.z = -ransac_model_coefficients_.at(2)*ransac_model_coefficients_.at(3);

  plane_orgin.x() = plane_vector_intersect_.x;
  plane_orgin.y() = plane_vector_intersect_.y;
  plane_orgin.z() = plane_vector_intersect_.z;

  plane_normal_vector.x() = ransac_model_coefficients_.at(0);
  plane_normal_vector.y() = ransac_model_coefficients_.at(1);
  plane_normal_vector.z() = ransac_model_coefficients_.at(2);

  center_frustum_vector.x() = center_frustum_.x;
  center_frustum_vector.y() = center_frustum_.y;
  center_frustum_vector.z() = center_frustum_.z;

  distance_scalar = (plane_orgin.dot(plane_normal_vector))/
                    (center_frustum_vector.dot(plane_normal_vector));

  std::cout << "distance_scalar: " << distance_scalar << std::endl;

  plane_vector_intersect = center_frustum_vector * distance_scalar;

  std::cout << "plane_vector_intersect: " << plane_vector_intersect << std::endl;

  intersect_point_.values[0] = plane_vector_intersect.x(); //plane_vector_intersect
  intersect_point_.values[1] = plane_vector_intersect.y();
  intersect_point_.values[2] = plane_vector_intersect.z();
  intersect_point_.values[3] = 0.1;

  plane_frustum_vector_intersect_.x = plane_vector_intersect.x();
  plane_frustum_vector_intersect_.y = plane_vector_intersect.y();
  plane_frustum_vector_intersect_.z = plane_vector_intersect.z();

  if (second_ransac_model_coefficients_.at(2)>0){ // TODO(simon) required.
    pose_vector_end_point_.x = plane_vector_intersect.x() + second_ransac_model_coefficients_.at(0);
    pose_vector_end_point_.y = plane_vector_intersect.y() + (-1*first_ransac_model_coefficients_.at(2));
    pose_vector_end_point_.z = plane_vector_intersect.z() + second_ransac_model_coefficients_.at(2);
  }
  else if (ransac_model_coefficients_.at(2)<0){

  }

//  std::cout << "intersect_point_: " << "x: " <<intersect_point_.values[0]
//                                    << "y: " <<intersect_point_.values[1]
//                                    << "z: " <<intersect_point_.values[2] << std::endl;
//  ransac_model_coefficients_;
//  center_frustum_;

}
void PoseEstimation::calculate_rotation_units() {


}



void PoseEstimation::log_data(uint32_t frame) {
  if (enable_logger_ && ransac_model_coefficients_.size() > 1 && tvecs_.size() >= 1 && rvecs_.size() >= 1){
    std::ofstream LoggerFile(std::filesystem::current_path().parent_path() / "log/data_out.csv", std::ios_base::app | std::ios_base::out);
    LoggerFile << frame << ","
               << plane_frustum_vector_intersect_.x << ","
               << plane_frustum_vector_intersect_.y << ","
               << plane_frustum_vector_intersect_.z << ","
               << second_ransac_model_coefficients_.at(0) << ","
               << (-1*first_ransac_model_coefficients_.at(2)) << ","
               << second_ransac_model_coefficients_.at(2) << ","
               << converted_ground_truth_vector_.at(0) << ","
               << converted_ground_truth_vector_.at(1) << ","
               << converted_ground_truth_vector_.at(2) << ","
               << converted_ground_truth_vector_.at(3) << ","
               << converted_ground_truth_vector_.at(4) << ","
               << converted_ground_truth_vector_.at(5)
               << std::endl;
    LoggerFile.close();
  }
  else {
    std::ofstream LoggerFile(std::filesystem::current_path().parent_path() / "log/data_out.csv", std::ios_base::app | std::ios_base::out);
    LoggerFile << frame <<std::endl;
    LoggerFile.close();
  }

}
