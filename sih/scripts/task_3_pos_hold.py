#!/usr/bin/python
'''
*team Id:		7360
*Author List:	Shruti Umat, Swamiprasad Amin, Sakshi Rathore, Rahul Paneriya
*Filename:		7360_pos_hold.py
Theme:			Hungry Bird
Functions:		class Edrone
					arm(self), 
					disarm(self), 
					whycon_callback(self, msg)
					altitude_set_pid(self, alt), 
					roll_set_pid(self,alt),
					pitch_set_pid(self, pit), 
					yaw_set_pid(self, yw)
					droneYaw(self, yw)
					setRange(self)
					pid(self)
Global 
variables:		none		
'''
from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time

class Edrone():
	"""docstring for Edrone"""
	'''
	*Function Name:
	*Input:
	*Output:
	*Logic:
	*Example Call:
	'''
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [0.0,0.0,0.0,0.0]	

		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		self.setpoint = [0.0, 0.0, 20.0, 69] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

		self.flag = 1

		#Declaring a cmd of message type PlutoMsg and initializing values
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		# self.cmd.plutoIndex = 0


		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [7.566, 9.486, 63.48, 9]		
		self.Ki = [0.064, 0.072, 0, 0]
		self.Kd = [147.3, 179.4, 0, 90.39]


		#-----------------------Add other required variables for pid here ----------------------------------------------
		self.prev_values = [0,0,0,0]
		self.max_values = [1700,1800,1800,1800]
		self.min_values = [1300,1200,1200,1200]
		self.sum_of_error = [0.0, 0.0, 0.0, 0.0]
		self.output = [0.0, 0.0, 0.0, 0.0]
		self.iterm = [0,0,0,0]






		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0,0] where corresponds to [pitch, roll, throttle, yaw]
		#		 Add variables for limiting the values like self.max_values = [1800,1800,1800,1800] corresponding to [pitch, roll, throttle, yaw]
		#													self.min_values = [1200,1200,1200,1200] corresponding to [pitch, roll, throttle, yaw]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.10 # in seconds





		self.error_pub = [0.0, 0.0, 0.0, 0.0]
		self.zero_line = 0

		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------
		self.error_pub[0] = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.error_pub[1] = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.error_pub[2] = rospy.Publisher('/alt_error', Float64, queue_size=1)
		self.error_pub[3] = rospy.Publisher('/yaw_error', Float64, queue_size=1)
		self.zero_line = rospy.Publisher('/zero_line', Float64, queue_size=1)



		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------
		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/pid_tuning_yaw',PidTune,self.yaw_set_pid)
		rospy.Subscriber('/drone_yaw',Float64, self.droneYaw)
		rospy.Subscriber('yaw_value', Int32, self.droneYaw)



		#------------------------------------------------------------------------------------------------------------

		self.arm() #ARMING THE DRONE



	'''
	*Function Name: altitude_set_pid(self, yw)
	*Input:			msg from topic /pid_tuning_altitude
	*Output:		sets Kp, Ki, Kd of altitude to that from 
					gui tuning after multiplying by a scale factor
	*Logic:
	*Example Call:	it is a callback function which gets 
					invoked when e_drone subscribes to the topic
					/pid_tuning_altitude
	'''
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.3

	'''
	*Function Name: pitch_set_pid(self, pit)
	*Input:			msg from topic /pid_tuning_yaw
	*Output:		sets Kp, Ki, Kd of pitch to that from 
					gui tuning after multiplying by a scale factor
	*Logic:
	*Example Call:	it is a callback function which gets 
					invoked when e_drone subscribes to the topic
					/pid_tuning_pitch
	'''
	def pitch_set_pid(self, pit):
		self.Kp[0] = pit.Kp * 0.006
		self.Ki[0] = pit.Ki * 0.008
		self.Kd[0] = pit.Kd * 0.3

	'''
	*Function Name: roll_set_pid(self, rl)
	*Input:			msg from topic /pid_tuning_roll
	*Output:		sets Kp, Ki, Kd of roll to that from 
					gui tuning after multiplying by a scale factor
	*Logic:
	*Example Call:	it is a callback function which gets 
					invoked when e_drone subscribes to the topic
					/pid_tuning_roll
	'''
	def roll_set_pid(self, rl):
		self.Kp[1] = rl.Kp * 0.006
		self.Ki[1] = rl.Ki * 0.008
		self.Kd[1] = rl.Kd * 0.3

	'''
	*Function Name: yaw_set_pid(self, yw)
	*Input:			msg from topic /pid_tuning_yaw
	*Output:		sets Kp, Ki, Kd of yaw to that from 
					gui tuning after multiplying by a scale factor
	*Logic:
	*Example Call:	it is a callback function which gets 
					invoked when e_drone subscribes to the topic
					/pid_tuning_yaw
	'''
	def yaw_set_pid(self, yw):
		self.Kp[3] = yw.Kp * 0.006 
		self.Ki[3] = yw.Ki * 0.0008
		self.Kd[3] = yw.Kd * 0.03


	'''
	*Function Name: disarm(self)
	*Input:			none
	*Output:		disarms the drone
	*Logic:			disarms the drone by setting rcAUX4 to 1100
	*Example Call:	e_drone.disarm()
	'''
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	'''
	*Function Name: arm(self)
	*Input:			none
	*Output:		arms the drone
	*Logic:			arms the drone by first disarming & then
					setting rcThrottle to 1000 and others to 1500
	*Example Call:	e_drone.arm()
	'''
	def arm(self):

		self.disarm()
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)


	''' 
	*Function Name: whycon_callback(self,msg)
	*Input:			msg of topic /whycon/poses
	*Output:		sets drone position
	*Logic:			PoseArray type of msg contains position
					within poses array, accessing it 
	*Example Call: The function gets executed each time when /whycon node publishes /whycon/poses 
	'''
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z

	
	'''
	*Function Name: droneYaw(self, yw)
	*Input:			yaw data from topic /drone_yaw
	*Output:		sets yaw value of drone as received from the topic
	*Logic:
	*Example Call:  it is a callback function which gets 
					invoked when e_drone subscribes to the topic
					/drone_yaw
	'''
	def droneYaw(self, yw):
		if self.flag == 1:
			self.setpoint[3] = yw.data
			self.flag = 0 
			
		self.drone_position[3] = yw.data

	'''
	*Function Name: setRange()
	*Input:			none
	*Output:		limits the values of drone command and output within max_values and min_values
	*Logic:			if drone's pitch, roll throttle or yaw is out of the range
					(lower than min_values or higher than max_values), then set it to min_value or max_value 
					respectively
	*Example Call:	e_drone.setRange()
	'''
	def setRange(self):
		'''	for i in range (0, 4):
			if self.output[i] > self.max_values[i] :
				self.output[i] = self.max_values[i]
			elif self.output[i] < self.min_values[i] :
				self.output[i] = self.min_values[i]
		'''
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

	'''
	*Function Name: pid(self)
	*Input:			none
	*Output:		computes change in error, iterms, output in all axes, 
					publishes error and sets the range of drone's pitch, roll, yaw and throttle
	*Logic:			calculates error, iterms and final pid output using known formulas
					calls the function setRange() to limit drone output values
	*Example Call:	e_drone.pid()
	'''
	def pid(self):

		error = [0.0, 0.0, 0.0, 0.0]
		change_in_error = [0.0, 0.0, 0.0, 0.0]

		for i in range (0, 4):
			error[i] = self.setpoint[i] - self.drone_position[i]
			self.error_pub[i].publish(error[i])
			change_in_error[i] = error[i] - self.prev_values[i]
			self.sum_of_error[i] = self.sum_of_error[i] + error[i]
			self.iterm[i] = self.iterm[i] + (error[i] * self.Ki[i])
			self.output[i] = (self.Kp[i] * error[i]) + self.iterm[i] + (self.Kd[i] * change_in_error[i])
			self.prev_values[i] = error[i] #storing current error in prev error

		print error
		self.cmd.rcPitch = 1500 + self.output[0]
		self.cmd.rcRoll = 1500 + self.output[1]
		self.cmd.rcThrottle = 1500 - self.output[2]
		self.cmd.rcYaw = 1500 + self.output[3]
		self.setRange()
		print self.cmd
		self.command_pub.publish(self.cmd)
		self.zero_line.publish(0.0)
		rospy.sleep(self.sample_time)
		'''
		if error[3] > 2:
			self.cmd.rcYaw = 1500 + self.output[3]
			self.cmd.rcThrottle = 1500 - self.output[2]
			self.setRange()
			self.command_pub.publish(self.cmd)
		else:
			#self.cmd.rcPitch = 1500 - self.output[0]
			#self.cmd.rcRoll = 1500 - self.output[1]
			self.cmd.rcThrottle = 1500 - self.output[2]
			self.cmd.rcYaw = 1500 + self.output[3]
			self.setRange()
			print self.cmd
			self.command_pub.publish(self.cmd)
			self.zero_line.publish(0.0)

		'''
		 
		#pid must NOT run continously


if __name__ == '__main__':

	e_drone = Edrone() #Declaring an object of class Edrone

	while not rospy.is_shutdown():
		e_drone.pid() #pid running till node is shutdown
