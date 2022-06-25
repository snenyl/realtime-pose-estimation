<div align="center"><h1>Realtime Pose Estimation</h1> </div>


[//]: # (<img src="assets/logo.png" width="350">)

## Introduction

This is part of the master’s thesis: "_A Machine Learning and Point Cloud Processing based Approach for Object Detection 
and Pose Estimation: Design, Implementation, and Validation_", available at [Link to thesis].

The 

[Bullet points]
## Demo video

<div align="center">

[<img src="assets/warehouse_test_thumbnail.png" width="80%">](https://www.youtube.com/watch?v=HvKInx1uoBw)
<figcaption align = "center"><b>Figure 1: Realtime pallet selection with point cloud extraction.</b></figcaption>

</div>

## Object Detection

The object detection algorithm is the YOLOX-S model from the YOLOX repository, which is transfer learned on the 
Logistics Objects in Context (LOCO) dataset. The final version is optimized with Intel OpenVINO and implemented together
with the pose estimation in C++. [Table 1](#table_1) present the training results for only pallet.

<center>

| **Model** | **Parameters** | **Dataset** | **AP** | **AP50** | **AP75** | **AP_S** | **AP_M** | **AP_L** | **Inference time** |
|:---------:|:--------------:|:-----------:|:------:|:--------:|:--------:|:--------:|:--------:|:--------:|:------------------:|
|   YOLOX-S |      9.0 M     |     LOCO    |  24.0% |   53.2%  |   17.2%  |   7.9%   |   24.3%  |   40.6%  |       6.74 ms      |
<figcaption align = "center"><b> <a name="table_1">Table 1:</a> Training results for only pallet's from the LOCO dataset, trained on an NVIDIA GeForce RTX 3060 Laptop GPU.</b></figcaption>
</center>



## Pose Estimation

The pose estimation is performed using the object detection algorithm and point cloud data. The object detection is only
the relevant point of the pallet used where two RANSAC operations are performed. The first plane uses only ground floor 
points, while the remaining pallet points are used for the second plane. A center vector from the camera is used to find
the 3D position where the vector intersects the pallet front plane.
While the pallet orientation is directly from the estimated from the front plane.




### Evaluation Demo of Moving and Standstill test using a AprilTag as ground truth 

<div style="width: 100%; overflow: hidden;">
     <div style="width: 48%; float: left;">

[<img src="assets/april_tag_test_thumbnail.png" width="100%"> ](https://youtu.be/BfEB9jjqpF0)
<figcaption align = "center"><b>Figure 2: Realtime pose estimation moving.</b></figcaption>


</div>
     <div style="margin-left: 52%;"> 

[<img src="assets/april_tag_test_thumbnail.png" width="100%">](https://youtu.be/-xybFGRdweY)
<figcaption align = "center"><b>Figure 3: Realtime pose estimation standstill.</b></figcaption>


</div>
</div>


## Future Work

- Improve the robustness of the system by implementing more checks, so the system is capable of failing softly.
- Find the pose using only the pallet front plane, thus increasing the range of the system. 

## License

[//]: # (All content included in this Git repository is under the license:)

[//]: # ()
[//]: # (Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International)

[//]: # ()
[//]: # (You are free to:)

[//]: # (* <b> Share </b> — copy and redistribute the material in any medium or format)

[//]: # (* <b> Adapt </b> — remix, transform, and build upon the material)

[//]: # ()
[//]: # (Under the following terms:)

[//]: # (* <b> Attribution </b> — You must give appropriate credit, provide a link to the license, and indicate if changes were made. You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.)

[//]: # (* <b> NonCommercial </b> — You may not use the material for commercial purposes.)

[//]: # (* <b> ShareAlike </b> — If you remix, transform, or build upon the material, you must distribute your contributions under the same license as the original.)

[//]: # (* <b> No additional restrictions </b> — You may not apply legal terms or technological measures that legally restrict others from doing anything the license permits.)



<p align="center">
<a rel="license" href="http://creativecommons.org/licenses/by/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by/4.0/">Creative Commons Attribution 4.0 International License</a>.