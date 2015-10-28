#!/usr/bin/env python

from __future__ import division
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
import mraa as m
import time
#import serial as ser
import serial

class RemoteHandler:
	def __init__(self):
		#ser.Serial('/dev/ttyO2', 57600)
		#ser = serial.Serial('/dev/ttyO2', 57600)
		self.dev = serial.Serial('/dev/ttyO2', 57600)
		rospy.init_node('remote_reading_handler');

	def publish(self):
		#pub = rospy.Publisher('remote_readings', Float64MultiArray, queue_size=10)
		pub = rospy.Publisher('remote_readings', Float64MultiArray, queue_size=1)
		rate = rospy.Rate(50) #10Hz
		a = [0.0, 0.0]
		dim = MultiArrayDimension()
		dim.size = len(a)
		dim.label = "REMOTEmsg"
		dim.stride = len(a)
		
		apub = Float64MultiArray()
		apub.data = a
		apub.layout.dim.append(dim)
		apub.layout.data_offset = 0

		while not rospy.is_shutdown():
			#UART_read = ser.readline()
			#UART_read = ser.Serial.readline()
			UART_read = self.dev.readline()
			steer_cmd = int(UART_read[0:4])
			throt_cmd = int(UART_read[4:8])
			a = [throt_cmd, steer_cmd]
			rospy.loginfo(a)
			apub.data = a
			pub.publish(apub)
			rate.sleep()

if __name__ == '__main__' :
	try :
		rem = RemoteHandler();
		rem.publish();
	except rospy.ROSInterruptException:
		pass
