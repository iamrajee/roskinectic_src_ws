#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image


def image_callback(img_msg):
    rospy.loginfo(img_msg.header)# log some info about the image topic

def simple_image_sub():
    rospy.init_node("simple_image_sub",anonymous=True)
    sub_image = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    try:
        simple_image_sub()
    except rospy.ROSInterruptException:
        pass