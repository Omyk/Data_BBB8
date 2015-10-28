#!/usr/bin/env python

from __future__ import division
import sys
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
import serial

class RobocapeController:
	def __init__(self):
		#initialize node
		rospy.init_node('RoboCapeController')
		rospy.loginfo(rospy.get_caller_id() + 'Initializing RoboCapeController node')

		#initialize serial communication (with remote)
		self.dev = serial.Serial('/dev/ttyO2', 115200)

		#initialize global variables
		self.Roll = 0
		self.RollRate = 0
		self.ThrottleREMOTE = 0
		self.SteeringREMOTE = 0

		self.RollCmd = 0
		self.RollRateCmd = 0
		self.SteeringCmd = 0

		self.Kp = 10
		self.Kv = 0.1

	def subscribe(self):
		#subscribe to IMU messages
		rospy.Subscriber('imu_readings', Imu, self.callbackIMU)
		#subscribe to REMOTE messages
		#rospy.Subscriber('remote_readings', Temperature, self.callbackREMOTE)
		#rospy.spin()

	def callbackIMU(self, msg):
		#save IMU messages into global variables
		self.Roll = msg.orientation.x
		self.RollRate = msg.angular_velocity.x
		rospy.loginfo([self.Roll, self.RollRate])

	#def callbackREMOTE(self, msg):
		#save REMOTE messages into global variables
		#RollCmd_temp = self.RollCmd
		#self.ThrottleREMOTE = msg.temperature
		#self.SteeringREMOTE = msg.variance
		#self.RollCmd = ((msg.variance-1000)/5.555555555555)-90
		#self.RollRateCmd = self.RollCmd - RollCmd_temp
		#RollCmd_temp = self.RollCmd
		#rospy.loginfo([self.ThrottleREMOTE, self.SteeringREMOTE])
		
	def publish(self):
		pub = rospy.Publisher('remote_readings', Temperature, queue_size=1)
		rate = rospy.Rate(100) #50Hz
		msg_temp = Temperature();
		msg_temp.header.frame_id = "robocape"
		
		while not rospy.is_shutdown():
			#now = rospy.get_rostime()
			while self.dev.inWaiting() <= 51:
				pass
			UART_read = self.dev.readline()
			self.dev.flushInput()
			self.SteeringREMOTE = int(UART_read[0:4])
			self.ThrottleREMOTE = int(UART_read[4:8])
			
			RollCmd_temp = self.RollCmd
			self.RollCmd = ((self.SteeringREMOTE-1000)/5.555555555555)-90
                	self.RollRateCmd = self.RollCmd - RollCmd_temp
			rospy.loginfo([self.ThrottleREMOTE, self.SteeringREMOTE])

			now = rospy.get_rostime()
			msg_temp.header.stamp.secs = now.secs
			msg_temp.header.stamp.nsecs = now.nsecs
			msg_temp.temperature = self.ThrottleREMOTE

			#controller
			self.SteeringCmd = self.Kp*(-self.Roll + self.RollCmd) + self.Kv*(-self.RollRate + self.RollRateCmd)			

			SteeringCmdPulse = ((self.SteeringCmd+90)*5.55555555555)+1000-50
	
			if SteeringCmdPulse > 1700:
				msg_temp.variance = 1700
			elif SteeringCmdPulse < 1300:
				msg_temp.variance = 1300
			else:
				msg_temp.variance = SteeringCmdPulse			

			pub.publish(msg_temp)
			#rospy.loginfo("Publishing :")
			rospy.loginfo(msg_temp.variance)
			rate.sleep()

	def control(self):
		self.subscribe()
		self.publish()
		rospy.spin()

if __name__ == '__main__' :
	try:
		controller = RobocapeController()
		controller.control()
		#controller.subscribe()
		#controller.publish()
	except rospy.ROSInterruptException:
		pass
