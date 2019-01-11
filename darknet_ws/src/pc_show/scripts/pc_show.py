#!/usr/bin/env python
#  -*- coding: utf-8

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int8


class PcShower():
    def __init__(self):
        rospy.init_node("PC_shower")
        rospy.loginfo("Waiting for image topic...")
        rospy.wait_for_message('/camera/rgb/image_color', Image)
        # 这里不需要定义rawimage_sub,如果需要后面不再订阅这个话题,需要定义.再使用取消
        self.raw_image = rospy.Subscriber("/camera/rgb/image_color", Image, self.getCVimage)
        self.raw_roi = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.show_roi)
        self.tracking_target = None
        self.roi_region = []

        self.object_list = ["t", "car"]
        self.object_choose = 0
        rospy.Subscriber('/cmd/change_target', Int8, self.change_target, queue_size=1)

    def getCVimage(self, raw_image):
        try:
            # aviod the roi_region being changed by other thread when using it to draw


            TX_frame = CvBridge().imgmsg_to_cv2(raw_image, "bgr8")
            # rospy.loginfo("transfer ros_image to cv_image")
            if self.tracking_target != None:
                tmp_tracking = self.tracking_target
                self.tracking_target = None
                cv2.rectangle(TX_frame, (tmp_tracking.xmin, tmp_tracking.ymin), \
                              (tmp_tracking.xmax, tmp_tracking.ymax), (0, 0, 255), 2)
            if self.roi_region != []:
                tmp_roi = self.roi_region
                self.roi_region = []
                for other in tmp_roi:
                    cv2.rectangle(TX_frame, (other.xmin, other.ymin), \
                                  (other.xmax, other.ymax), (0, 255, 0), 2)

            # if len(tmp_roi) == 1:
            #     track_target = tmp_roi[0]
            #     cv2.rectangle(TX_frame, (track_target.xmin, track_target.ymin), \
            #                   (track_target.xmax, track_target.ymax), (0, 255, 0), 2)
            # elif len(tmp_roi) > 1:
            #     track_target = tmp_roi[-1]
            #     cv2.rectangle(TX_frame, (track_target.xmin, track_target.ymin), \
            #                   (track_target.xmax, track_target.ymax), (0, 255, 0), 2)
            #     for other in tmp_roi[:-1]:
            #         cv2.rectangle(TX_frame, (other.xmin, other.ymin), \
            #                       (other.xmax, other.ymax), (0, 0, 255), 2)

            cv2.namedWindow("TX_view")
            cv2.imshow("TX_view", TX_frame)
            cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)

    def show_roi(self, msg):
        if len(msg.bounding_boxes) > 1:
            print("There are {a} objects, and select the one with highest probability".format(a=len(msg.bounding_boxes)))
        all_target_regions = [box for box in msg.bounding_boxes if box.Class == self.object_list[int(self.object_choose)]]
        sorted(all_target_regions, key=lambda box: box.probability)
        self.tracking_target = all_target_regions[-1]
        self.roi_region = [box for box in msg.bounding_boxes if box != self.tracking_target]
        self.roi_region = self.roi_region + all_target_regions[:-1]

    def change_target(self, msg):
        self.object_choose = int(msg.data)

if __name__ == '__main__':
    try:
        PcShower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("PcShower node terminated.")




# roi_region = []
# global roi_region
#
# def getCVimage(raw_image):
#     try:
#         global roi_region
#
#         # aviod the roi_region being changed by other thread when using it to draw
#         tmp_roi = roi_region
#         roi_region = []
#
#         TX_frame = CvBridge().imgmsg_to_cv2(raw_image, "bgr8")
#         rospy.loginfo("transfer ros_image to cv_image")
#         if len(tmp_roi) == 1:
#             track_target = tmp_roi[0]
#             cv2.rectangle(TX_frame, (track_target.xmin, track_target.ymin), \
#                           (track_target.xmax, track_target.ymax), (0, 255, 0), 2)
#         elif len(tmp_roi) > 1:
#             track_target = tmp_roi[-1]
#             cv2.rectangle(TX_frame, (track_target.xmin, track_target.ymin), \
#                           (track_target.xmax, track_target.ymax), (0, 255, 0), 2)
#             for other in tmp_roi[:-1]:
#                 cv2.rectangle(TX_frame, (other.xmin, other.ymin), \
#                               (other.xmax, other.ymax), (0, 0, 255), 2)
#         cv2.namedWindow("TX_view")
#         cv2.imshow("TX_view", TX_frame)
#         cv2.waitKey(1)
#
#     except CvBridgeError as e:
#         print(e)
#
# def show_roi(msg):
#     global roi_region
#     if len(msg.bounding_boxes) > 1:
#         print("There are more than one objects, and select the one with highest probability")
#     roi_region = msg.bounding_boxes
#     sorted(roi_region, key=lambda box: box.probability)
#
#
# rospy.init_node("PC_shower")    # 没有这句话就不能继续!!!必须要初始化一个节点
# rospy.loginfo("Waiting for image topic...")
# rospy.wait_for_message('/camera/rgb/image_color', Image)
# # 这里不需要定义rawimage_sub,如果需要后面不再订阅这个话题,需要定义.再使用取消
# raw_image = rospy.Subscriber("/camera/rgb/image_color", Image, getCVimage)
# raw_roi = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, show_roi)
#
# rospy.spin()



