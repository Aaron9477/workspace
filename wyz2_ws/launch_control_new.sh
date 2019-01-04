#!/bin/bash
sudo bash ~/jetson_clocks.sh
#gnome-terminal --window -e 'bash -c "roslaunch turtlebot_bringup minimal.launch;exec bash"'
#sleep 2s
#gnome-terminal --window -e 'bash -c "roslaunch freenect_launch freenect.launch;exec bash"'
#sleep 2s
source ~/WYZ/wyz2_ws/devel/setup.bash
gnome-terminal --window -e 'bash -c "rosrun fasterrcnn2 demo_camera_ros.py;exec bash"'
sleep 2s
source ~/WYZ/wyz_ws/devel/setup.bash
gnome-terminal --window -e 'bash -c "rosrun tld run_tld_ros -p /home/ubuntu/WYZ/wyz_ws/src/tld/parameters.yml;exec bash"'
source ~/WYZ/wyz2_ws/devel/setup.bash
rosrun object_follower object_follower.py
