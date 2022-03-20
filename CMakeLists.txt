cmake_minimum_required(VERSION 3.21)
project(realtime_pose_estimation)

set(CMAKE_CXX_STANDARD 17)
set(OpenCV_DIR "/usr/include/opencv4") # Select OpenCV with contrib
# /usr/include/opencv4
# /opt/intel/openvino_2021.4.752/opencv/include

find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED) # COMPONENTS aruco PATHS "/usr/lib/x86_64-linux-gnu/"

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
