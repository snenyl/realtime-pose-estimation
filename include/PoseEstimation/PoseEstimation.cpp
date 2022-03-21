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

//  std::cout << "X: " << detection_output_struct_.x << std::endl;
//  std::cout << "Y: " << detection_output_struct_.y << std::endl;
//  std::cout << "Width: " << detection_output_struct_.width << std::endl;
//  std::cout << "Height: " << detection_output_struct_.height << std::endl;
//  std::cout << "Confidence: " << detection_output_struct_.confidence << std::endl;

  calculate_3d_crop();


//  edit_pointcloud();
  view_pointcloud();


  const int w = image.as<rs2::video_frame>().get_width();
  const int h = image.as<rs2::video_frame>().get_height();

  cv::Mat cv_image(cv::Size(w, h), CV_8UC3, (void*)image.get_data(), cv::Mat::AUTO_STEP);
  cv::cvtColor(cv_image,cv_image,cv::COLOR_BGR2RGB);

  image_ = cv_image;

  object_detection_object_.run_object_detection(image_);
  calculate_aruco();
  calculate_pose();

  int iterator;
  iterator++;
  if (iterator > 10){ // TODO(simon) Debugging iterator.
//    std::cout << "hello!" << std::endl;
    iterator = 0;
  }



  cv::imshow("Output",cv_image);
  cv::waitKey(33);


}
void PoseEstimation::setup_pose_estimation() {
//  rosbag_path_ = std::filesystem::current_path().parent_path() / "data/20220227_151307.bag";
  rosbag_path_ = std::filesystem::current_path().parent_path() / "data/20220319_112907.bag"; //Standstill both aruco and detection
//  rosbag_path_ = std::filesystem::current_path().parent_path() / "data/20220319_112823.bag"; //Nice

  if (load_from_rosbag){
    std::cout << "Loaded rosbag: " << rosbag_path_ << std::endl;
//    p.stop(); // TODO(simon) check if stream is running. p.get_active_profile().get_device()
    rs2::config cfg;
    cfg.enable_device_from_file(rosbag_path_);
    p.start(cfg);

  } else p.start();

//  rs2_get_video_stream_intrinsics()

  set_camera_parameters();
  
  object_detection_object_.setup_object_detection();
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_ = viewer;

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


  if (!rvecs.empty() && !tvecs.empty()){
    std::stringstream rotation;
    std::stringstream translation;

    rvecs_ = rvecs;
    tvecs_ = tvecs;

    float rvecs_deg[3];

    for (int i = 0; i < 3; ++i) {
      rvecs_deg[i] = rvecs.at(0)[i]*57.2958;
    }

//    rotation << rvecs.at(0);
    rotation << "[" << rvecs_deg[0] << ", " << rvecs_deg[1] << ", " << rvecs_deg[2] << "]";
    translation << tvecs.at(0);
//    std::cout << rvecs.size() << "\n" << tvecs.size() << "\n" << object_points.size() << std::endl;
//    std::cout << rvecs.at(0) << "\n" << tvecs.at(0) << std::endl;
    cv::putText(image_,rotation.str(),cv::Point(50,50),cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,255,0),2,false);
    cv::putText(image_,translation.str(),cv::Point(50,100),cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,255,0),2,false);
    cv::aruco::drawAxis(image_,example_camera_matrix_,example_dist_coefficients_,rvecs,tvecs,0.1);
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
//  pcl::PointCloud<pcl::PointXYZ>::Ptr local_pallet(new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Matrix4f camera_pose;
  Eigen::Matrix4f rotation_red_pos_ccw;
  Eigen::Matrix4f rotation_green_pos_cw;
  Eigen::Matrix4f cam2robot;

  cam2robot <<   0, 0, 1, 0,
                 0,-1, 0, 0,
                 1, 0, 0, 0,
                 0, 0, 0, 1;

  camera_pose << -1, 0,  0, 0,
                  0, 1,  0, 0,
                  0, 0, -1, 0,
                  0, 0,  0, 1;

  float rot_red_rad = 0.35;
  float rot_green_rad = -0.10;

  rotation_red_pos_ccw << std::cos(rot_red_rad),-std::sin(rot_red_rad), 0,0,
                          std::sin(rot_red_rad), std::cos(rot_red_rad), 0,0,
                          0, 0, 1,  0,
                          0, 0,  0, 1;

  rotation_green_pos_cw << std::cos(rot_green_rad), 0,-std::sin(rot_green_rad), 0,
                          0,                         1,                          0, 0,
                          std::sin(rot_green_rad), 0, std::cos(rot_green_rad), 0,
                          0,                         0,                          0, 1;

  frustum_filter.setInputCloud(cloud_ptr_);
  frustum_filter.setCameraPose(camera_pose*cam2robot*rotation_red_pos_ccw*rotation_green_pos_cw);
  frustum_filter.setNearPlaneDistance(0);
  frustum_filter.setFarPlaneDistance(15);
  frustum_filter.setVerticalFOV(fov_v_rad_ * 57.2958);
  frustum_filter.setHorizontalFOV(fov_h_rad_ * 57.2958);
//  frustum_filter.filter(*local_pallet);


  std::cout << "cloud_ptr_ size: " << cloud_ptr_->height * cloud_ptr_->width << std::endl;
//  std::cout << "local_pallet size: " << local_pallet->height * local_pallet->width << std::endl;
//  std::cout << "local_pallet size MB: " << sizeof(local_pallet) << std::endl;
  std::cout << "cloud_ptr_ size: " << *cloud_ptr_ << std::endl;

  frustum_filter.setInputCloud(pcl_points_);


}
void PoseEstimation::view_pointcloud() {

    if (first_run_){
    viewer_->setBackgroundColor (0, 0, 0);
    viewer_->addCoordinateSystem (1);
    viewer_->initCameraParameters();
    viewer_->setCameraPosition(0, 10, 0,    0, 0, 0,   0, 0, 1);
    first_run_ = false;
  }

  viewer_->removeAllShapes();
  viewer_->removeAllPointClouds();
  viewer_->removeCoordinateSystem("aruco_marker",0);
  viewer_->addPointCloud(pcl_points_);

  if (!rvecs_.empty() && !tvecs_.empty()){
//    std::cout << "RVECS: " << rvecs_.at(0) << std::endl;
//    std::cout << "TVECS: " << tvecs_.at(0) << std::endl;
//    viewer_->addCoordinateSystem(0.535,0.128196, 0.0394752, 0.815717,"aruco_marker",0);

    rot_trans_matrix_.translation()[0] = static_cast<float>(tvecs_.at(0)[0]);
    rot_trans_matrix_.translation()[1] = static_cast<float>(tvecs_.at(0)[1]);
    rot_trans_matrix_.translation()[2] = static_cast<float>(tvecs_.at(0)[2]);

//    rot_trans_matrix_.linear() = ( Eigen::AngleAxisd(3.1415 / 6, Eigen::Vector3d::UnitY()) *
//                                   Eigen::AngleAxisd(3.1415 / 6, Eigen::Vector3d::UnitX()) ).toRotationMatrix();


//    rot_trans_matrix.fromPositionOrientationScale((1,1,1),(1,1,1),(1,1,1));

    viewer_->addCoordinateSystem(0.535,rot_trans_matrix_,"aruco_marker",0);
//    viewer_->addCoordinateSystem(0.535,tvecs_.at(0)[0], tvecs_.at(0)[1], tvecs_.at(0)[2],"aruco_marker",0);
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



  viewer_->spinOnce(100);

}
void PoseEstimation::calculate_3d_crop() {

  std::vector<Eigen::Vector2d> detection_point_vec(4);

  Eigen::Vector2d image_center(zed_k_matrix_[2],
                               zed_k_matrix_[3]);

  detection_point_vec.at(0).x() = detection_output_struct_.x;
  detection_point_vec.at(0).y() = detection_output_struct_.y;

  detection_point_vec.at(1).x() = detection_output_struct_.x + detection_output_struct_.width;
  detection_point_vec.at(1).y() = detection_output_struct_.y;

  detection_point_vec.at(2).x() = detection_output_struct_.x;
  detection_point_vec.at(2).y() = detection_output_struct_.y + detection_output_struct_.height;

  detection_point_vec.at(3).x() = detection_output_struct_.x + detection_output_struct_.width;
  detection_point_vec.at(3).y() = detection_output_struct_.y + detection_output_struct_.height;

  for (int i = 0; i < 4; ++i) {
    detection_from_image_center_.emplace_back((detection_point_vec.at(i) - image_center) / zed_k_matrix_[0]);
    detection_from_image_center_.at(i).y() *= -1; // TODO(simon) This is a hack to invert the y-axis.
    square_frustum_detection_points_.emplace_back(detection_from_image_center_.at(i).x()*detection_vector_scale_,
                                                  detection_from_image_center_.at(i).y()*detection_vector_scale_,
                                                  -1*detection_vector_scale_);
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

//  std::cout << "Vector 0: " << vector_0 << std::endl;
//  std::cout << "Vector 1: " << vector_1 << std::endl;
//  std::cout << "Vector 2: " << vector_2 << std::endl;
//  std::cout << "Vector 3: " << vector_3 << std::endl;
//  std::cout << "vector_center: " << vector_center << std::endl;

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

}
