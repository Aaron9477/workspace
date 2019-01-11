#!/usr/bin/env python
#  -*- coding: utf-8

import rospy
from std_msgs.msg import Int8


class TargetToCar():
    def __init__(self):
        rospy.init_node("target_to_car")
        self.change_pub = rospy.Publisher('/cmd/change_target', Int8, queue_size=1)

        self.rate = rospy.get_param("~rate", 10)
        r = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            self.change_pub.publish(0)
            r.sleep()

if __name__ == '__main__':
    try:
        TargetToCar()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("target_to_car node terminated.")

