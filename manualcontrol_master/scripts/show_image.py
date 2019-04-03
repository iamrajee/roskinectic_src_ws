#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def image_callback(img_msg):
    rospy.loginfo(img_msg.header)# log some info about the image topic
    try:
        # Initialize the CvBridge class
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Show the converted image
    show_image(cv_image)

def show_image(bgr_img):
    rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
    # b,g,r = cv2.split(bgr_img)       # get b,g,r
    # rgb_img = cv2.merge([r,g,b])     # switch it to rgb

    cv2.imshow("Image Window", rgb_img)
    cv2.waitKey(3)
def simple_image_sub():
    rospy.init_node("simple_image_sub",anonymous=True)
    sub_image = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

    # cv2.namedWindow("Image Window", 1)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    try:
        simple_image_sub()
    except rospy.ROSInterruptException:
        # close all windows
        cv2.destroyAllWindows()
        # pass