cmake_minimum_required(VERSION 3.21)
project(realtime_pose_estimation)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

add_library(pose_estimation
            include/PoseEstimation/PoseEstimation.h
            src/PoseEstimation.cpp
            )

include_directories(
            include/PoseEstimation
            ${OpenCV_INCLUDE_DIRS}
)

add_executable(realtime_pose_estimation src/main.cpp)

target_link_libraries(realtime_pose_estimation
                      pose_estimation
                      ${realsense2_LIBRARY}
                      ${OpenCV_LIBRARIES}
)
