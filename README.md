<div align="center"><h1>Real-time Pose Estimation</h1> </div>

This is part of the master’s thesis: "_A Machine Learning and Point Cloud Processing based Approach for Object Detection 
and Pose Estimation: Design, Implementation, and Validation_", available at [TBD link to thesis].

By combining an RGB image and point cloud data is the system capable of detecting the object's pose by using object detection,
RANSAC and vector operations. This work is based on the [YOLOX](https://github.com/Megvii-BaseDetection/YOLOX) algorithm and
[Logistics Objects in Context (LOCO)](https://github.com/tum-fml/loco) dataset from 2021 and 2020, respectively.

## Demo Video

<div align="center">

[<img src="assets/warehouse_test_thumbnail.png" width="80%">](https://www.youtube.com/watch?v=HvKInx1uoBw)
<figcaption align = "center"><b><a name="figure_1">Figure 1:</a> Real-time pallet selection with point cloud extraction.</b></figcaption>

</div>

[Figure 1](#figure_1) Is a video of the object detection filtering out the pallet of the point cloud. No pose estimation
is performed as the depth data from the Intel RealSense l515 does not capture the ground of the pallet.

## Object Detection

The object detection algorithm is the YOLOX-S model from the [YOLOX](https://github.com/Megvii-BaseDetection/YOLOX) 
repository, which is transfer learned on the [LOCO](https://github.com/tum-fml/loco) dataset. The final version is 
optimized with [Intel OpenVINO](https://github.com/openvinotoolkit/openvino) and implemented together with the pose 
estimation in C++. A total of two models has been created from the pallet dataset and [Table 1](#table_1) present the 
YOLOX-S training results for only pallet and pallet void.

<center>

|      **Model**      | **Parameters** | **Dataset** | **$AP$** | **$AP_{50}$** | **$AP_{75}$** | **$AP_S$** | **$AP_M$** | **$AP_L$** | **Inference time** |
|:-------------------:|:--------------:|:-----------:|:--------:|:-------------:|:--------------:|:----------:|:----------:|:----------:|:------------------:|
| yolox_s_only_pallet |      9.0 M     |     LOCO    |  24.0%   |     53.2%     |     17.2%      |    7.9%    |   24.3%    |   40.6%    |       6.74 ms      |
| yolox_s_pallet_void |      9.0 M     |     LOCO    |   0.2%   |     0.2%      |      0.2%      |    0.0%    |    0.7%    |    0.0%    |       6.96 ms      |

<figcaption align = "center"><b> <a name="table_1">Table 1:</a> Training results for only_pallet and pallet_void from the LOCO dataset, trained on an NVIDIA GeForce RTX 3060 Laptop GPU.</b></figcaption>
</center>

&nbsp;

### Only pallet detection model

The YOLOX algorithm has been tested on synthetic pallets in Unreal Engine as shown in [Figure 2](#figure_2).

<div align="center">

[<img src="assets/unreal_synthetic_pallet.png" width="80%">](https://youtu.be/XEYaCEHEH3g)
<figcaption align = "center"><b><a name="figure_2">Figure 2:</a> Synthetic pallet detection video test in Unreal Engine using YOLOX-S trained on the LOCO dataset.</b></figcaption>

&nbsp;

</div>

### Only pallet void detection model


## Pose Estimation
The pose estimation is performed using the object detection algorithm and point cloud data.

Object detection filters out only the relevant points of the pallet where two RANSAC operations are performed.
The first plane uses only ground floor points, while the remaining pallet points are used for the second plane.
A center vector from the camera is used to find the 3D position where the vector intersects the pallet front plane, 
while the pallet orientation is directly from the estimated front plane. [Figure 3](#figure_3) explains the system 
outputs in the PCL viewer.
<div align="center">

[<img src="assets/3d_explain.png" width="80%"> ](#figure_3)
<figcaption align = "center"><b><a name="figure_3">Figure 3:</a> Vector and point explanation from the PCL 3D viewer.</b></figcaption>

&nbsp;

</div>

### Evaluation Demo Video

The evaluation of the pose estimation system is done with an AprilTag (Type: 25h9), which can directly output its pose using
functions from the OpenCV contrib library. A total of two tests have been performed. Moving test where the camera rotates
around from 0 to -90 degrees and a standstill test where accuracy and precision are evaluated.
The distance from the pallet is two-meter for both tests. [Figure 4](#figure_4) and [Figure 5](#figure_5) links to a video.


<div align="center">

[<img src="assets/april_tag_test_moving_thumbnail.png" width="80%"> ](https://youtu.be/BfEB9jjqpF0)
<figcaption align = "center"><b><a name="figure_4">Figure 4:</a> Real-time moving test evaluated with an AprilTag.</b></figcaption>

&nbsp;

[<img src="assets/april_tag_test_thumbnail.png" width="80%">](https://youtu.be/-xybFGRdweY)
<figcaption align = "center"><b><a name="figure_5">Figure 5:</a> Real-time standstill test evaluated with an AprilTag.</b></figcaption>

</div>

## Requirements

| **Package**     | **Minimum version** | **Info**                                                                                |
|-----------------|---------------------|-----------------------------------------------------------------------------------------|
| realsense2      | 2.50.0              | From the [Intel® RealSense™ SDK 2.0](https://github.com/IntelRealSense/librealsense).   |
| OpenCV          | 4.2.0               | [OpenCV_contib](https://github.com/opencv/opencv_contrib) is required for aruco module. |
| InferenceEngine | 2021.4.752          | From the [OpenVINO toolkit](https://github.com/openvinotoolkit/openvino).               |
| ngraph          | N/A                 | Required for InferenceEngine, and part of openVINO.                                     |
| PCL             | 1.10.0              | From [PointCloudLibrary/pcl](https://github.com/PointCloudLibrary/pcl).                 |
## Future Work<a name="future_work"></a>

- Improve the robustness of the system by implementing 
- Increasing the range of the pose estimation by using only a single plane and not requiring the ground plane
 (Dynamic depending on number of remaining points after extraction). 
- Detecting pallet holes.
- Make all the vector operations in a single matrix operation.
- Add configuration.
- Add [TensorRT](https://github.com/NVIDIA/TensorRT) based object detection class library for optimized inference on NVIDIA hardware.

## Known Issues<a name="known_issues"></a>

- The colors in the image sometimes switch between RGB and BGR. Unknown if this is an issue in the code or in 
 the imported rosbag. The issue is shown in the [demo at 01:26](https://youtu.be/HvKInx1uoBw?t=86). 