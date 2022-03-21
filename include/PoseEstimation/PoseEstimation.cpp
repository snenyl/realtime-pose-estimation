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
  rosbag_path_ = std::filesystem::current_path().parent_path() / "data/20220319_112823.bag";

  if (load_from_rosbag){
    std::cout << "Loaded rosbag: " << rosbag_path_ << std::endl;
//    p.stop(); // TODO(simon) check if stream is running. p.get_active_profile().get_device()
    rs2::config cfg;
    cfg.enable_device_from_file(rosbag_path_);
    p.start(cfg);

  } else p.start();

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
  cv::aruco::estimatePoseSingleMarkers(markerCorners_,0.175,example_camera_matrix_,example_dist_coefficients_,rvecs,tvecs,object_points);
  // TODO(simon) Set marker size as parameter. 0.175 0.535

//  std::cout << rvecs.size() << "\n" << tvecs.size() << "\n" << object_points.size() << std::endl;
//  std::cout << rvecs.at(0) << "\n" << tvecs.at(0) << std::endl;

  if (rvecs.size() > 0 && tvecs.size() > 0){
    std::stringstream rotation;
    std::stringstream translation;

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
  viewer_->addPointCloud(pcl_points_);
  viewer_->spinOnce(100);

}
