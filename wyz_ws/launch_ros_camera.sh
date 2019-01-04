#!/bin/bash
source ~/LPH/devel/setup.bash
gnome-terminal --window -e "roslaunch rbx1_vision usb_cam.launch video_device:=/dev/video0"
sleep 2s
source ~/WYZ/wyz2_ws/devel/setup.bash
gnome-terminal --window -e "rosrun fasterrcnn2 demo_camera_ros.py"
sleep 2s
source ~/WYZ/wyz_ws/devel/setup.bash
rosrun tld run_tld_ros -p /home/zq610/Desktop/parameters.yml
