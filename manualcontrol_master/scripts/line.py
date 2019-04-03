#!/usr/bin/env python
import rospy
import cv2
import time
import imutils
import numpy as np
from copy import copy,deepcopy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


def image_callback(img_msg):
    # rospy.loginfo(img_msg.header)# log some info about the image topic
    try:
        # Initialize the CvBridge class
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Show the converted image
    find_line(cv_image)

def find_line(frame):
    fy,fx,_ = np.shape(frame)
    delta = 60
    tl= fx/2 - delta
    tr= fx/2 + delta
    # rospy.loginfo("Size : " + str(np.shape(frame)))
    # rospy.loginfo("fy,fx = "+str(fy)+","+str(fx))

    crop_img = deepcopy(frame[fy/2:fy, 0:fx])
	#cv2.imshow('crop_img',crop_img)                            #<<--

	# Convert to grayscale
    gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
	#cv2.imshow('gray',gray)                                    #<<--

	# Gaussian blur
    blur = cv2.GaussianBlur(gray,(5,5),0)
    #cv2.imshow('blur',blur)                                    #<<--
    
    # Color thresholding
    ret,thresh = cv2.threshold(blur,60,255,cv2.THRESH_BINARY_INV)
    #cv2.imshow('thresh',thresh)                                #<<--
    
    # Find the contours of the frame
    contours = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)
    contours = imutils.grab_contours(contours)
    
    line_info_pub = rospy.Publisher("line_info",String,queue_size=1)
    
    if len(contours) > 0:           
        fright=0
        fleft=0

        c = max(contours, key=cv2.contourArea) #CONTOUR WITH MAX AREA
    
        M = cv2.moments(c)
    
    
    
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    
    
        # DISPLAYING CX AND CY
        cv2.line(crop_img,(cx,0),(cx,720),(255,0,0),1)
        cv2.line(crop_img,(0,cy),(1280,cy),(255,0,0),1)
    
        # DISPLAYING LEFT, RIGHT, FORWARD
        cv2.line(crop_img,(tl,0),(tl,fy/2),(0,0,255),2)
        cv2.line(crop_img,(tr,0),(tr,fy/2),(0,0,255),2)
    
        # DRAWING OUTLING AROUND CONTOUR
        cv2.drawContours(crop_img, contours, -1, (0,255,0), 1)
    
    
        if cx >= tr:
            # sright()
            fright =1
            # print("Turn Right")
            line_info_pub.publish("Turn Right")
        elif cx < tr and cx > tl:
            # forward()
            # print("On Track!")
            line_info_pub.publish("On Track!")
        elif cx <= tl:
            # sleft()
            fleft =1
            # print("Turn Left")
            line_info_pub.publish("Turn Left!")
    else: #IF no countor
        '''
        if fright==1:
            sleft()
        elif fleft==1:
            sright()
        else:'''
        # Stop()
        # print("I don't see the line")
        line_info_pub.publish("I don't see the line!!!")

        
                                   #<<--
	
	
	#rawCapture.truncate(0)# picamera only # clear the stream in preparation for the next frame
	
	#Breaking condition
	# key = cv2.waitKey(1) & 0xFF
	# if key == ord("q"):
    #         Stop()
	#     break

    # cv2.imshow("frame", frame)
    # cv2.imshow('final_frame',crop_img) 
    cv2.waitKey(3)

    image_pub = rospy.Publisher("image/line",Image,queue_size=1)
    image_pub.publish(bridge.cv2_to_imgmsg(crop_img, "bgr8"))



def line():
    rospy.init_node("line",anonymous=True)
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

    # cv2.namedWindow("Image Window", 1)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    try:
        bridge = CvBridge()
        line()
    except rospy.ROSInterruptException:
        # close all windows
        cv2.destroyAllWindows()
        # pass