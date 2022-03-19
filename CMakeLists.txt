cmake_minimum_required(VERSION 3.21)
project(realtime_pose_estimation)

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED PATHS "/usr/lib/x86_64-linux-gnu/")

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(include/PoseEstimation)

include_directories(
#            include/PoseEstimation
            ${OpenCV_INCLUDE_DIRS}
)

add_executable(realtime_pose_estimation src/main.cpp)

target_link_libraries(realtime_pose_estimation
                      pose_estimation
                      ${realsense2_LIBRARY}
                      ${OpenCV_LIBS}
)

