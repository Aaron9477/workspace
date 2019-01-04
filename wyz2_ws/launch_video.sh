#!/bin/bash
source /home/zq610/WYZ/wyz_ws/devel/setup.bash
gnome-terminal --window --tab -e "roscore"
sleep 2s
gnome-terminal --window --tab -e "rosrun fasterrcnn demo_video.py"
sleep 4s
source /home/zq610/WYZ/wyz2_ws/devel/setup.bash	# in different workspace
rosrun tld_origin run_tld -p /home/zq610/Desktop/parameters.yml -s /home/zq610/WYZ/media/video/TestVideo.mp4
