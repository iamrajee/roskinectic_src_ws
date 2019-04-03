#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty, UInt16

import time
from time import *

servo_delay = 0.025

def sweep(s,e):
    if ((e - s) > 0):
        change = 1
    else:
        change = -1

    while(s != e):
        servo_pub.publish(s)
        sleep(servo_delay)
        s+=change

def lookAround_cb(msg):
    sweep(90,0)
    sweep(0,180)
    sweep(180,89)

def lookAround_sub():
    rospy.init_node("lookAround",anonymous=True)
    rospy.Subscriber("lookAround", Empty, lookAround_cb)
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_pub = rospy.Publisher("servo", UInt16, queue_size=10)
        lookAround_sub()
    except rospy.ROSInterruptException:
        pass