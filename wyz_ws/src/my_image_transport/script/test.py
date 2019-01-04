#!/usr/bin/env python
#  -*- coding: utf-8

import rospy
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
import cv2


def getCVimage(raw_image):
    try:
        global TX1_frame
        TX1_frame = CvBridge().imgmsg_to_cv2(raw_image, "bgr8")
        print("transfer ros_image to cv_image")
    except CvBridgeError as e:
        print (e)

def show_roi(roi):
    global TX1_frame
    cv2.rectangle(TX1_frame, (roi.x_offset, roi.y_offset), (roi.x_offset+roi.width, roi.y_offset+roi.height), (0, 255, 0), 5)


rospy.loginfo("Waiting for image topic...")
rospy.wait_for_message('/camera/rgb/image_raw', Image)

# 这里不需要定义rawimage_sub,如果需要后面不再订阅这个话题,需要定义.再使用取消
rawimage_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, getCVimage, queue_size=1)
rospy.Subscriber('tld_roi', RegionOfInterest, show_roi, queue_size=1)

cv2.namedWindow("TX1_view")
global TX1_frame
cv2.imshow("TX1_view", TX1_frame)

