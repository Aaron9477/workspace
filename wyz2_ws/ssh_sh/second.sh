#!/bin/bash
#ssh ubuntu@192.168.1.65 "source ~/WYZ/wyz2_ws/devel/setup.bash; rosrun fasterrcnn2 demo_camera_ros.py"
#ssh ubuntu@192.168.1.65 "echo 'ubuntu' | sudo -S ./jetson_clocks.sh; export DISPLAY=:0.0; source ~/WYZ/wyz2_ws/devel/setup.bash; rosrun fasterrcnn2 demo_camera_ros_original.py"
ssh ubuntu@192.168.1.65 "echo 'ubuntu' | sudo -S ./jetson_clocks.sh; source ~/WYZ/wyz2_ws/devel/setup.bash; rosrun fasterrcnn2 demo_camera_ros.py"

#ssh ubuntu@192.168.1.65 "echo 'ubuntu' | sudo -S ./jetson_clocks.sh"

