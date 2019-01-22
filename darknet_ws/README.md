## Tracking and following with yolov3-tiny with TX2 turtlebot
---
### This project learn from the following project
```
@misc{bjelonicYolo2018,
  author = {Marko Bjelonic},
  title = {{YOLO ROS}: Real-Time Object Detection for {ROS}},
  howpublished = {\url{https://github.com/leggedrobotics/darknet_ros}},
  year = {2016--2018},
}
```
### hardware you need
---
Two turtlebotv2 (one as the controlled and one as the tracked), one turtlebot3 (as the tracked), one PC controler station (for sending commend and recieving view from the controlled), one TX2 (as the processor of the controlled), two TX1 or NUC (as the processor of the tracked)
### software you need
---
- ubuntu 16.04. cuda8.0, cudnn6.5
- OpenCV (computer vision library),
- boost (c++ library)
### how to use it
---
Log in the TX2 on the controlled with ssh, and run the following command
```
roslaunch darknet_ros detection_control_TX2.launch
```
On the PC, run the following command
```
rosrun camera_read test_image_subscriber
```