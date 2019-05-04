#!/usr/bin/env python

import rospy
import roslib
import tf

from geometry_msgs.msg import PoseArray
from aruco_msgs.msg import MarkerArray


#Defining a class
class Marker_detect():

	def __init__(self):
		rospy.init_node('marker_detection',anonymous=False) # initializing a ros node with name marker_detection

		self.whycon_marker = {}	# Declaring dictionaries
		self.aruco_marker = {}

		rospy.Subscriber('/whycon/poses',PoseArray,self.whycon_data)	# Subscribing to topic
		rospy.Subscriber('/aruco_marker_publisher/markers',MarkerArray,self.aruco_data)	# Subscribing to topic
		


	# Callback for /whycon/poses
	def whycon_data(self,msg):
		for i in range(0,len(msg.poses)):
			pos_x = round(msg.poses[i].position.x,3)
			pos_y = round(msg.poses[i].position.y,3)
			pos_z = round(msg.poses[i].position.z,3)
			self.whycon_marker[i] = [pos_x,pos_y,pos_z]




	# Callback for /aruco_marker_publisher/markers
	def aruco_data(self,msg):
		for i in range(0,len(msg.markers)):
			aruco_id = msg.markers[i].id
			orient_x = round(msg.markers[i].pose.pose.position.x,3)
			orient_y = round(msg.markers[i].pose.pose.position.y,3)
			orient_z = round(msg.markers[i].pose.pose.position.z,3)
			#orient_w = round(msg.markers[i].pose.pose.orientation.w,3)
			self.aruco_marker[aruco_id] = [orient_x,orient_y,orient_z]

		# Above is the method in which index is set according to the id of the aruco marker
		# However, in task0, those who have set index starting from 0 independent of the aruco id is also considered correct. However the above method is the correct method which you should follow in future.
		


		# Printing the detected markers on terminal
		print "\n"
		print "WhyCon_marker",self.whycon_marker
		print "ArUco_marker",self.aruco_marker




if __name__=="__main__":

	marker = Marker_detect()

	
	while not rospy.is_shutdown():
		rospy.spin()