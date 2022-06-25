<div align="center"><h1>Realtime Pose Estimation</h1> </div>


[//]: # (<img src="assets/logo.png" width="350">)

## Introduction

This is part of the master’s thesis: "_A Machine Learning and Point Cloud Processing based Approach for Object Detection 
and Pose Estimation: Design, Implementation, and Validation_", available at [Link to thesis].

[Bullet points]
## Demo video

<div align="center">

[<img src="assets/warehouse_test_thumbnail.png" width="80%">](https://www.youtube.com/watch?v=HvKInx1uoBw)
<figcaption align = "center"><b>Figure 1: Realtime pallet selection with point cloud extraction.</b></figcaption>

</div>

## Object Detection

The object detection algorithm is from the YOLOX. [Cite yolox]
The used dataset is from Logistics Objects in Context (LOCO) [cite ]

Optimized with OpenVINO.

Training results. 

## Pose Estimation

### Demo 

 <div style="width: 100%; overflow: hidden;">
     <div style="width: 48%; float: left;">

[<img src="assets/april_tag_test_thumbnail.png" width="48%"> ](https://youtu.be/BfEB9jjqpF0){target="_blank"}
<figcaption align = "center"><b>Figure 2: Realtime pose estimation moving.</b></figcaption>


</div>
     <div style="margin-left: 52%;"> 

[<img src="assets/april_tag_test_thumbnail.png" width="48%">](https://youtu.be/-xybFGRdweY){target="_blank"}
<figcaption align = "center"><b>Figure 3: Realtime pose estimation standstill.</b></figcaption>


</div>
</div>


## Future Work

- Improve the robustness of the system.
- Find the pose without using the ground plane. 

## License