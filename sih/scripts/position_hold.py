#!/usr/bin/env python
'''
		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
		/yaw_error				/pid_tuning_yaw
								/drone_yaw
'''
# Importing the required libraries
from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from aruco_msgs.msg import MarkerArray
from pid_tune.msg import PidTune
import rospy
import time
	
class position_hold():
	def __init__(self):
		rospy.init_node('position_hold')

		# ================== INITIALISE ========================#
		self.cmd = PlutoMsg()
		self.pid_sample_time = 0.10 #in sec #how frequently should pid run #NOTE: stimulation step time is 0.050 sec
		self.zero_line_value = 0.0  #?
		# [x,y,z,yaw_value]
		self.droneAruco_pos = [0.0,0.0,0.0,0.0]	
		self.setpoint = [-8.39, 4.98, 27.92, 0.0] #goal
		#[pitch, roll, throttle, yaw]
		self.output = [0.0, 0.0, 0.0, 0.0]
		self.prev_values = [0,0,0,0]
		self.max_values = [1700,1700,1800,1800]
		self.min_values = [1300,1300,1200,1200]
		self.Kp = [5.13, 12, 23.1, 9] 
		self.Ki = [0.8 , 0, 0, 0.1496]
		self.Kd = [13.365, 58.5, 342.9, 112.83]

		self.proposnal = [0.0, 0.0, 0.0, 0.0]
		self.integral = [0.0, 0.0, 0.0, 0.0]
		self.differential = [0.0, 0.0, 0.0, 0.0]

		# ================== Publisher =========================#
		self.drone_command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		self.error_pub = [None,None,None,None] #list of publiser - simlpy initialised
		self.error_pub[0] = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.error_pub[1] = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.error_pub[2] = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.error_pub[3] = rospy.Publisher('/yaw_error', Float64, queue_size=1)
		self.zero_line_pub = rospy.Publisher('/zero_line', Float64, queue_size=1)

		# ========================= Subscriber ======================#
		rospy.Subscriber('/aruco_marker_publisher/markers',MarkerArray,self.aruco_cb)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_pid_cb)
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_pid_cb)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_pid_cb)
		rospy.Subscriber('/pid_tuning_yaw',PidTune,self.yaw_pid_cb)
		rospy.Subscriber('/drone_yaw',Float64, self.droneYaw_cb)

		#ARMING THE DRONE
		self.arm()

	def pid(self):
		self.proposnal = [0.0, 0.0, 0.0, 0.0] #reset to 0
		self.differential = [0.0, 0.0, 0.0, 0.0] #reset to 0
		for i in range (0, 4): #for [x,y,z,yaw]
			self.proposnal[i] = self.setpoint[i] - self.droneAruco_pos[i]	#p
			self.integral[i] = self.integral[i] + self.proposnal[i]			#I
			self.differential[i] = self.proposnal[i] - self.prev_values[i]	#D
			#(kp*P + ki*I + kd*D)
			self.output[i] = (self.Kp[i] * self.proposnal[i]) \
				+ (self.Ki[i] * self.integral[i]) + (self.Kd[i] * self.differential[i])
			self.prev_values[i] = self.proposnal[i]
			self.error_pub[i].publish(self.proposnal[i])
		self.cmd.rcPitch = 1500 - self.output[0] #pitch
		self.cmd.rcRoll = 1500 - self.output[1]	#roll
		self.cmd.rcThrottle = 1500 - self.output[2] #throttle
		self.cmd.rcYaw = 1500 + self.output[3]	#yaw
		self.limit()							#bound
		self.drone_command_pub.publish(self.cmd)#publish
		self.zero_line_pub.publish(self.zero_line_value)
		rospy.sleep(self.pid_sample_time)

	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.drone_command_pub.publish(self.cmd)
		rospy.sleep(1)

	def arm(self):
		self.disarm() #always disarm before arming
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000 # <------
		self.cmd.rcAUX1 = 1500 
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		self.cmd.plutoIndex = 0
		self.drone_command_pub.publish(self.cmd)
		rospy.sleep(1)

	def aruco_cb(self,msg):
		self.droneAruco_pos[0] = round(msg.markers[self.cmd.plutoIndex].pose.pose.position.x,3)
		self.droneAruco_pos[1] = round(msg.markers[self.cmd.plutoIndex].pose.pose.position.y,3)
		self.droneAruco_pos[2] = round(msg.markers[self.cmd.plutoIndex].pose.pose.position.z,3)
	def pitch_pid_cb(self, pit):
		self.Kp[0] = pit.Kp * 0.006
		self.Ki[0] = pit.Ki * 0.008
		self.Kd[0] = pit.Kd * 0.003
	def roll_pid_cb(self, rl):
		self.Kp[1] = rl.Kp * 0.06
		self.Ki[1] = rl.Ki * 0.008
		self.Kd[1] = rl.Kd * 0.0375
	def altitude_pid_cb(self,alt):
		self.Kp[2] = alt.Kp * 0.06 # This is just for an example. You can change the fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3
	def yaw_pid_cb(self, yw):
		self.Kp[3] = yw.Kp * 0.006
		self.Ki[3] = yw.Ki * 0.0008
		self.Kd[3] = yw.Kd * 0.03
	def droneYaw_cb(self, yw):
		self.droneAruco_pos[3] = yw.data
	
	def limit(self):
		for i in range (0, 4):
			if self.output[i] > self.max_values[i] :
				self.output[i] = self.max_values[i]
			elif self.output[i] < self.min_values[i] :
				self.output[i] = self.min_values[i]
		if self.cmd.rcPitch > self.max_values[0]:
		 	self.cmd.rcPitch = self.max_values[0]
		elif self.cmd.rcPitch < self.min_values[0]:
		 	self.cmd.rcPitch = self.min_values[0]
		if self.cmd.rcRoll > self.max_values[1]:
		 	self.cmd.rcRoll = self.max_values[1]
		elif self.cmd.rcRoll < self.min_values[1]:
		 	self.cmd.rcRoll = self.min_values[1]
		if self.cmd.rcThrottle > self.max_values[2]:
		 	self.cmd.rcThrottle = self.max_values[2]
		elif self.cmd.rcThrottle < self.min_values[2]:
		 	self.cmd.rcThrottle = self.min_values[2]
		if self.cmd.rcYaw > self.max_values[3]:
		 	self.cmd.rcYaw = self.max_values[3]
		elif self.cmd.rcYaw < self.min_values[3]:
		 	self.cmd.rcYaw = self.min_values[3]

if __name__ == '__main__':
	position_hold_class = position_hold()
	while not rospy.is_shutdown():
		position_hold_class.pid()