#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty

def toggle_led():
    rospy.init_node("toggle_led",anonymous=True)
    pub = rospy.Publisher("toggle_led", Empty, queue_size=10)
    # rate = rospy.Rate(10) #in Hz
    # sleep for duration
    d = rospy.Duration(2, 0) #in sec
    while not rospy.is_shutdown():
        empty_str = Empty()
        pub.publish(empty_str)
        # rate.sleep()
        rospy.sleep(d)

if __name__ == '__main__':
    try:
        toggle_led()
    except rospy.ROSInterruptException:
        pass