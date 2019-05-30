#!/usr/bin/env python
import rospy
import roslib
import tf
from geometry_msgs.msg import PoseArray
from aruco_msgs.msg import MarkerArray

class arucopose():
	def __init__(self):
		rospy.init_node('arucopose',anonymous=False)
		self.aruco_marker = {}
		rospy.Subscriber('/aruco_marker_publisher/markers',MarkerArray,self.aruco_cb)

	def aruco_cb(self,msg):
		for i in range(0,len(msg.markers)):
			aruco_id = msg.markers[i].id
			orient_x = round(msg.markers[i].pose.pose.position.x,3)
			orient_y = round(msg.markers[i].pose.pose.position.y,3)
			orient_z = round(msg.markers[i].pose.pose.position.z,3)
			#orient_w = round(msg.markers[i].pose.pose.orientation.w,3)
			self.aruco_marker[aruco_id] = [orient_x,orient_y,orient_z]

		print("\nArUco_marker:",self.aruco_marker)

if __name__=="__main__":
	marker = arucopose()
	while not rospy.is_shutdown():
		rospy.spin()