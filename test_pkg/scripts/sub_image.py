#!/usr/bin/env python
"""OpenCV feature detectors with ros CompressedImage Topics in python.

This example subscribes to a ros topic containing sensor_msgs 
CompressedImage. It converts the CompressedImage into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as CompressedImage topic.
"""

import sys, time
import numpy as np
import cv2
from scipy.ndimage import filters
import rospy, roslib

from sensor_msgs.msg import CompressedImage
# from cv_bridge import CvBridge, CvBridgeError # We do not use cv_bridge it does not support CompressedImage in python

VERBOSE=True

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        self.image_pub = rospy.Publisher("/ardrone/image_raw/compressed",CompressedImage,queue_size = 1)
        # self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber("/rosout/image_raw/compressed",CompressedImage, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to /ardrone/image_raw/compressed"


    def callback(self, ros_data):
        print("callback")
        '''Callback function of subscribed topic.Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        
        #### Feature detectors using CV2 #### 
        # "","Grid","Pyramid" + ("FAST","GFTT","HARRIS","MSER","ORB","SIFT","STAR","SURF")
        method = "GridFAST"
        feat_det = cv2.FeatureDetector_create(method)
        time1 = time.time()

        # convert np image to grayscale
        featPoints = feat_det.detect(
            cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY))
        time2 = time.time()
        if VERBOSE :
            print '%s detector found: %s points in: %s sec.'%(method,
                len(featPoints),time2-time1)

        for featpoint in featPoints:
            x,y = featpoint.pt
            cv2.circle(image_np,(int(x),int(y)), 3, (0,0,255), -1)
        
        cv2.imshow('cv_img', image_np)
        cv2.waitKey(2)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        self.image_pub.publish(msg)
        
        #self.subscriber.unregister()

def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('image_feature', anonymous=True)
    ic = image_feature()
    
    try:
        print("before")
        rospy.spin()
        print("after")
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)