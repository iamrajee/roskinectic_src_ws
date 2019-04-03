#!/usr/bin/env python
import cv2
import time,os
import imutils
import numpy as np
from copy import copy,deepcopy
from PIL import Image
import pickle

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

#global variable
count=0
watched=[]

def image_callback(img_msg):
    # rospy.loginfo(img_msg.header)# log some info about the image topic
    try:
        # Initialize the CvBridge class
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Show the converted image
    find_face(cv_image)

def find_face(frame):
    global count
    global watched
    im = deepcopy(frame)

    gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    faces=faceCascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5, minSize=(100, 100), flags=cv2.CASCADE_SCALE_IMAGE)

    face_info_pub = rospy.Publisher("face_info",String,queue_size=1)
    sense_pub = rospy.Publisher("sensehat",String,queue_size=1)
    speak_pub = rospy.Publisher("speak",String,queue_size=1)
    flag = 0
    for(x,y,w,h) in faces:
        flag = 1
	count = 0
        nbr_predicted_name = "unknown"
        nbr_predicted, conf = recognizer.predict(gray[y:y+h,x:x+w])
        cv2.rectangle(im,(x-50,y-50),(x+w+50,y+h+50),(225,0,0),2)
        if(nbr_predicted==0):
            nbr_predicted_name="rajee"
        elif(nbr_predicted==1):
            nbr_predicted_name="manu"

	face_info_pub.publish(nbr_predicted_name)
	sense_pub.publish("print="+nbr_predicted_name)
	if nbr_predicted_name not in watched:
		watched.append(nbr_predicted_name)
		speak_pub.publish("hi "+nbr_predicted_name)
        cv2.putText(im,str(nbr_predicted_name)+"--"+str(conf), (x,y+h),font, fontscale, fontcolor,3) #Draw the text
    if flag == 0:
        face_info_pub.publish("Noone")
	count+=1
	if count==1:
		sense_pub.publish("clear")
	elif count==10:
		watched=[]
    # cv2.imshow('im',cv2.resize(im,None,fx = 0.5,fy=0.5))
    cv2.waitKey(3)


    rgb_im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    image_pub = rospy.Publisher("/image/face",Image,queue_size=1)
    image_pub.publish(bridge.cv2_to_imgmsg(cv2.resize(rgb_im,None,fx = 0.5,fy=0.5), "bgr8"))


def face_recognition():
    rospy.init_node("face_recognition",anonymous=True)
    rospy.Subscriber("camera0/usb_cam/image_raw", Image, image_callback)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == "__main__":
    try:
        recognizer = cv2.face.LBPHFaceRecognizer_create()
        recognizer.read('/home/rajendra/git/opencv/mycode/Face-Recognition/trainer/trainer.yml')
        cascadePath = "/home/rajendra/git/opencv/data/haarcascades/haarcascade_frontalface_default.xml"
        faceCascade = cv2.CascadeClassifier(cascadePath)
        path = '/home/rajendra/git/opencv/mycode/Face-Recognition/dataSet'

        # cam = cv2.VideoCapture(0)
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontscale = 1
        fontcolor = (0,0, 255)

        bridge = CvBridge()
        face_recognition()
    except rospy.ROSInterruptException:
        # close all windows
        cv2.destroyAllWindows()
        # pass
