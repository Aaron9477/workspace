#!/usr/bin/python
# -*- coding: utf-8 

"""
    object_follower.py - Version 1.1 2013-12-20
    
    Follow a target published on the /roi topic using depth from the depth image.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int8
from math import copysign
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import thread
import time
import copy


class ObjectFollower():
    def __init__(self):
        rospy.init_node("object_follower")
                        
        rospy.on_shutdown(self.shutdown)
        
        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate)
        #self.last_time = time.time()
        #self.command = Twist()
        
        self.scale_roi = rospy.get_param("~scale_roi", 0.8)
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.4)
        # the lowest speed set as 0, because I want to make speed increase with steps
        self.min_linear_speed = rospy.get_param("~min_linear_speed", 0)
        self.max_rotation_speed = rospy.get_param("~max_rotation_speed", 1.5)
        self.min_rotation_speed = rospy.get_param("~min_rotation_speed", 0.5)

        # the percent of shifting
        self.x_threshold = rospy.get_param("~x_threshold", 0.2)
        # if move, it will go to in the goal_z_threshold
        # if it stops, it will move if the diff lager than move_z_threshold 
        self.goal_z_threshold = rospy.get_param("~z_threshold", 0.07)
        self.move_z_threshold = rospy.get_param("~z_threshold", 0.15)
        self.move_z_flag = True

        # the threshold of ignore the depth of point of background
        self.max_z = rospy.get_param("~max_z", 2)
        # the min of depth we get
        self.min_z = rospy.get_param("~min_z", 0.3)
        self.goal_z = rospy.get_param("~goal_z", 0.75)
        self.z_scale = rospy.get_param("~z_scale", 0.3)
        self.x_scale = rospy.get_param("~x_scale", 1.5)
        # self.gain = rospy.get_param("~gain", 2.0)
        # self.slow_down_factor = rospy.get_param("~slow_down_factor", 0.8)
        
	    self.last_linear_speed = 0
        self.roi = BoundingBoxes()

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.move_cmd = Twist()
        self.lock = thread.allocate_lock()
        
        self.image_width = 640
        self.image_height = 480
        
        self.cv_bridge = CvBridge()
        self.depth_array = None
        self.target_visible = False

        # add object list to be choosen to track
        self.object_list = ["t", "car"]
        self.object_choose = 0
        rospy.Subscriber('/cmd/change_target', Int8, self.change_target, queue_size=1)

        rospy.loginfo("Waiting for camera_info topic...")
        rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo)
        rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.get_camera_info)

        # Wait until we actually have the camera data
        while self.image_width == 0 or self.image_height == 0:
            rospy.sleep(1)
            
        rospy.loginfo("Waiting for depth_image topic...")
        rospy.wait_for_message('camera/depth/image_raw', Image)
        self.depth_subscriber = rospy.Subscriber("camera/depth/image_raw", Image, self.convert_depth_image, queue_size=1)

        rospy.Subscriber('/darknet_ros/found_object', Int8, self.get_detected_flag)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.set_cmd_vel)
        rospy.loginfo("Waiting for an ROI to track...")
        rospy.wait_for_message('/darknet_ros/found_object', Int8)

        rospy.loginfo("ROI messages detected. Starting follower...")
        
        # Begin the tracking loop
        while not rospy.is_shutdown():
            # Acquire a lock while we're setting the robot speeds
#            if time.time() - self.last_time > 0.4:
#		# there couldn't use '=', because this is a list
#                self.command = copy.deepcopy(self.move_cmd)
#                self.last_time = time.time()
#                print("change the commend")
            
            self.lock.acquire()
            try:
                if not self.target_visible:
                    self.move_cmd = Twist()
                else:
                    self.target_visible = False
                self.cmd_vel_pub.publish(self.move_cmd)
            finally:
                self.lock.release()
            r.sleep()


    def set_cmd_vel(self, msg):
        # Acquire a lock while we're setting the robot speeds
        #print("I got message from yolo, xmin:{a}, ymin:{b}, xmax:{c}, ymax:{d}".format \
        #          (a=msg.bounding_boxes[0].xmin, b=msg.bounding_boxes[0].ymin, c=msg.bounding_boxes[0].xmax, d=msg.bounding_boxes[0].ymax))
        self.lock.acquire()

        try:
            # choose the target with the class we want
            target_boxes = [box for box in msg.bounding_boxes if box.Class == self.object_list[int(self.object_choose)]]

            if len(target_boxes) <= 0:
                print("There is no object we want")
                self.move_cmd = Twist()
            else:
                if len(target_boxes) > 1:
                    #print("There are more than one objects, and select the one with highest probability")
                    sorted(target_boxes, key=lambda box : box.probability)
                    self.roi = target_boxes[-1]
                else:
                    # self.target_visible = True
                    self.roi = target_boxes[0]

                roi_width = int(self.roi.xmax - self.roi.xmin)
                roi_height = int(self.roi.ymax - self.roi.ymin)

                if roi_width <= 0 or roi_height <= 0:
                    self.target_visible = False
                    return

                # Get the min/max x and y values from the scaled ROI
                scaled_min_x = int(self.roi.xmin + roi_width * (1.0 - self.scale_roi) / 2.0)
                scaled_max_x = int(self.roi.xmax - roi_width * (1.0 - self.scale_roi) / 2.0)
                scaled_min_y = int(self.roi.ymin + roi_height * (1.0 - self.scale_roi) / 2.0)
                scaled_max_y = int(self.roi.ymax - roi_height * (1.0 - self.scale_roi) / 2.0)

                # Compute the displacement of the ROI from the center of the image
                target_offset_x = self.roi.xmin + roi_width / 2 - self.image_width / 2

                # avoid dividing zero
                try:
                    percent_offset_x = float(target_offset_x) / (float(self.image_width) / 2.0)
                except:
                    percent_offset_x = 0

                #print("the shifting of target is {a}".format(a=percent_offset_x))

                # Rotate the robot only if the displacement of the target exceeds the threshold
                if abs(percent_offset_x) > self.x_threshold:
                    # Set the rotation speed proportional to the displacement of the target
                    speed = self.x_scale * percent_offset_x
                    self.linear_speed = 0
                    if speed < 0:
                        direction = 1
                    else:
                        direction = -1
                    self.move_cmd.angular.z = direction * max(self.min_rotation_speed, min(self.max_rotation_speed, abs(speed)))
                    self.move_cmd.linear.x = 0
                    #print("the rotation speed is {a}".format(a=self.move_cmd.angular.z))

                else:
                    self.move_cmd.angular.z = 0

                    # Initialize a few depth variables
                    n_z = sum_z = mean_z = 0

                    # Get the average depth value over the ROI
                    # only use 7 points to reduce the calculation complexity
                    if scaled_max_x-scaled_min_x >= 7 and scaled_max_y-scaled_min_y >= 7:
                        for x in range(scaled_min_x, scaled_max_x, int((scaled_max_x-scaled_min_x)/7)):
                            for y in range(scaled_min_y, scaled_max_y, int((scaled_max_x-scaled_min_x)/7)):
                                try:
                                    # Get a depth value in millimeters
                                    z = self.depth_array[y, x]
                                    # Convert to meters
                                    z /= 1000.0
                                except:
                                    # It seems to work best if we convert exceptions to 0
                                    z = 0

                                # Check for values outside max range, maybe it is a background point
                                if z > self.max_z:
                                    continue
                                # Increment the sum and count
                                sum_z = sum_z + z
                                n_z += 1
                    else:
                        for x in range(scaled_min_x, scaled_max_x):
                            for y in range(scaled_min_y, scaled_max_y):
                                try:
                                    # Get a depth value in millimeters
                                    z = self.depth_array[y, x]
                                    # Convert to meters
                                    z /= 1000.0
                                except:
                                    # It seems to work best if we convert exceptions to 0
                                    z = 0

                                # Check for values outside max range
                                if z > self.max_z:
                                    continue
                                # Increment the sum and count
                                sum_z = sum_z + z
                                n_z += 1

                    if n_z:
                        mean_z = float(sum_z) / n_z

                        #print("the raw mean depth is {a}".format(a=mean_z))

                        mean_z = max(self.min_z, mean_z)
                        # Check the mean against the minimum range
                        '''if mean_z > self.min_z:
                            # Check the max range and goal threshold
                            if mean_z < self.max_z and (abs(mean_z - self.goal_z) > self.z_threshold):
                                speed = (mean_z - self.goal_z) * self.z_scale
                                self.move_cmd.linear.x = copysign(min(self.max_linear_speed, max(self.min_linear_speed, abs(speed))), speed)                  
                                print mean_z,speed      
                            else:
                                 # Otherwise stop the robot
                                 self.move_cmd.linear.x=0 '''
                        if abs(mean_z - self.goal_z) > self.goal_z_threshold:
                            if abs(mean_z - self.goal_z) > self.move_z_threshold or self.move_z_flag == True:
                                self.move_z_flag = True
                                target_speed = (mean_z - self.goal_z) * self.z_scale
				                # the speed should be changed slowly
                                if abs(target_speed - self.last_linear_speed) < 0.1:
                                    speed = (self.last_linear_speed + 0.02) if target_speed > self.last_linear_speed else (self.last_linear_speed - 0.02)
                                else:
                                    speed = target_speed
                                self.last_linear_speed = speed
                                if speed < 0:
                                    speed *= 1.5
                                self.move_cmd.linear.x = copysign(
                                    min(self.max_linear_speed, max(self.min_linear_speed, abs(speed))), speed)
                        else:
                            self.move_z_flag = False
                            self.move_cmd.linear.x = 0
                        #print("the linear speed is {a}".format(a=self.move_cmd.linear.x))

        finally:
            # Release the lock
            self.lock.release()

    def convert_depth_image(self, ros_image):
        try:
            # The depth image is a single-channel float32 image
            depth_image = self.cv_bridge.imgmsg_to_cv2(ros_image, "32FC1")
        except CvBridgeError, e:
            print e
        self.depth_array = np.array(depth_image, dtype=np.float32)

    def get_camera_info(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height

    def get_detected_flag(self, msg):
        self.target_visible = True if msg.data > 0 else False

    def change_target(self, msg):
        self.object_choose = int(msg.data)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Unregister the subscriber to stop cmd_vel publishing
        self.depth_subscriber.unregister()
        rospy.sleep(1)
        # Send an emtpy Twist message to stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)      

if __name__ == '__main__':
    try:
        ObjectFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Object follower node terminated.")

