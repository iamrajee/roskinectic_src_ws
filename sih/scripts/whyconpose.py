#!/usr/bin/env python
import rospy
import roslib
import tf
from geometry_msgs.msg import PoseArray
from aruco_msgs.msg import MarkerArray

class whyconpose():
	def __init__(self):
		rospy.init_node('whyconpose',anonymous=False)
		self.whycon_marker = {}
		rospy.Subscriber('/whycon/poses',PoseArray,self.whycon_cb)

	def whycon_cb(self,msg):
		for i in range(0,len(msg.poses)):
			pos_x = round(msg.poses[i].position.x,3) + 0.025
			pos_y = -round(msg.poses[i].position.y,3) - 0.017
			pos_z = 54.44-round(msg.poses[i].position.z,3)
			self.whycon_marker[i] = [pos_x,pos_y,pos_z]
		print("\nwhycon_marker:",self.whycon_marker)

if __name__=="__main__":
	marker = whyconpose()
	while not rospy.is_shutdown():
		rospy.spin()