#!/bin/bash
#ssh ubuntu@192.168.1.65 "export DISPLAY=:0.0; source ~/WYZ/wyz_ws/devel/setup.bash; rosrun tld run_tld_ros -p /home/zq610/Desktop/parameters.yml"
ssh ubuntu@192.168.1.65 "export DISPLAY=:0.0; source ~/WYZ/wyz_ws/devel/setup.bash; rosrun tld run_tld_ros_original -p ~/WYZ/wyz_ws/src/tld/parameters.yml"

