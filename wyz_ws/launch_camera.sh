#!/bin/bash
source /home/zq610/WYZ/wyz_ws/devel/setup.bash
gnome-terminal --window --tab -e "roscore"
sleep 2s
#test data
#gnome-terminal --window --tab -e "rosrun fasterrcnn talker_test"
gnome-terminal --window --tab -e "rosrun fasterrcnn demo_camera.py"
sleep 4s
rosrun tld run_tld -p /home/zq610/Desktop/parameters.yml
