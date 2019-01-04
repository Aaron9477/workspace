#!/usr/bin/env python

from beginner_tutorials.srv import AddTwoInts
import rospy
import tensorflow

def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    res = AddTwoInts()
    res.sum1 = req.a + req.b + req.a
    res.sum2 = req.a + req.b
    return res.sum1, res.sum2

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
