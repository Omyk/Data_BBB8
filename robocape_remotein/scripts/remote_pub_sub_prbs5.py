#!/usr/bin/env python

from __future__ import division
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import Temperature
#import mraa as m
import mraa
import time
#import serial as ser
import serial

class RemoteHandler:
	def __init__(self):
		#ser.Serial('/dev/ttyO2', 57600)
		#ser = serial.Serial('/dev/ttyO2', 57600)
		#self.dev = serial.Serial('/dev/ttyO2', 57600)
		self.dev = serial.Serial('/dev/ttyO2', 115200)
		rospy.init_node('remote_reading_handler');

		#Get all parameters from config (rosparam)
                name = 'engine'
                engine_output_pin = int(rospy.get_param('actuators/' + name + '/output_pin', 1))
                engine_board_pin = int(rospy.get_param('actuators/' + name + '/board_pin', 60))
                engine_period_us = int(1e6 / float(rospy.get_param('actuators/' + name + '/frequency', 60)))

                name = 'steering'
                steering_output_pin = int(rospy.get_param('actuators/' + name + '/output_pin', 1))
                steering_board_pin = int(rospy.get_param('actuators/' + name + '/board_pin', 62))
                steering_period_us = int(1e6 / float(rospy.get_param('actuators/' + name + '/frequency', 60)))

                #Initialize PWM
                self.dev1 = mraa.Pwm(engine_board_pin)
                self.dev1.period_us(engine_period_us)
                self.dev1.enable(True)
                self.dev1.pulsewidth_us(1500)

                self.dev2 = mraa.Pwm(steering_board_pin)
                self.dev2.period_us(steering_period_us)
                self.dev2.enable(True)
                self.dev2.pulsewidth_us(1500)

	def publish(self):
		#pub = rospy.Publisher('remote_readings', Float64MultiArray, queue_size=10)
		#pub = rospy.Publisher('remote_readings', Float64MultiArray, queue_size=1)
		pub = rospy.Publisher('remote_readings', Temperature, queue_size=1)
		rate = rospy.Rate(40) #10Hz

		msg_temp = Temperature();
		msg_temp.header.frame_id = "robocape"

		start_state = 0x7D0;
		lfsr = start_state;

		steer_low = 1440;
		steer_high= 1560;
		counterp = 0;
		steer_prbs = 1500
		#a = [0.0, 0.0]
		#dim = MultiArrayDimension()
		#dim.size = len(a)
		#dim.label = "REMOTEmsg"
		#dim.stride = len(a)
		
		#apub = Float64MultiArray()
		#apub.data = a
		#apub.layout.dim.append(dim)
		#apub.layout.data_offset = 0

		#cnt=0

		while not rospy.is_shutdown():
			#begin = time.time()
			#UART_read = ser.readline()
			#UART_read = ser.Serial.readline()
			#self.dev.flushInput()
			while self.dev.inWaiting() <= 11:
				pass
			UART_read = self.dev.readline()
			self.dev.flushInput()
			#if self.dev.inWaiting() > 80:
			#	self.dev.flushInput()
			#if self.dev.inWaiting() > 50:
			#	self.dev.flushInput()
			#	cnt=0
			#self.dev.flushInput()
			#rospy.loginfo(UART_read)
			steer_cmd = int(UART_read[0:4])
			throt_cmd = int(UART_read[4:8])
			#self.dev.flushInput()
			if counterp == 4 :
				bit = ((lfsr >> 0) ^ (lfsr >> 2)) & 1;
				lfsr = (lfsr >> 1) | (bit << 10);

				if bit == 1:
					steer_prbs = steer_high
				elif bit == 0:
					steer_prbs = steer_low
				else:
					steer_prbs = steer_cmd
				counterp = 0

			if steer_cmd > steer_high:
				steer_cmd = steer_cmd
			elif steer_cmd < steer_low:
				steer_cmd = steer_cmd
			else:
				steer_cmd = steer_prbs

			counterp = counterp + 1

			now = rospy.get_rostime()
			msg_temp.header.stamp.secs = now.secs
			msg_temp.header.stamp.nsecs = now.nsecs
			msg_temp.temperature = throt_cmd
			msg_temp.variance = steer_cmd

			self.dev1.pulsewidth_us(throt_cmd)
			self.dev2.pulsewidth_us(steer_cmd)

			pub.publish(msg_temp)
			rospy.loginfo([throt_cmd, steer_cmd])
			#a = [throt_cmd, steer_cmd]
			#rospy.loginfo(a)
			#apub.data = a
			#msg_joint.position = apub
			#pub.publish(msg_joint)
			#buffer = self.dev.inWaiting()
			#rospy.loginfo(buffer)
			#cnt=cnt+1
			#self.dev.flushInput()
			#length = time.time() - begin
			#rospy.loginfo(length)	
			rate.sleep()
			#length2 = time.time() - begin
			#rospy.loginfo(length2)

if __name__ == '__main__' :
	try :
		rem = RemoteHandler();
		rem.publish();
	except rospy.ROSInterruptException:
		pass
