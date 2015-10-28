#!/usr/bin/env python

from __future__ import division
import sys
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import Temperature
import mraa
import time

class ActuatorHandler:
	def __init__(self):
		rospy.init_node('actuators_handler')
		rospy.loginfo(rospy.get_caller_id() + 'Initializing actuators_handler node')

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

	def subscribe(self):
		rospy.Subscriber('remote_readings', Temperature, self.callback, queue_size=1, tcp_nodelay=True)
		rospy.spin()

	def callback(self, msg):
		self.dev1.pulsewidth_us(int(msg.temperature))
		self.dev2.pulsewidth_us(int(msg.variance))
		rospy.loginfo([msg.temperature, msg.variance])

if __name__ == '__main__' :
	try :
		actuators = ActuatorHandler()
		actuators.subscribe()
	except rospy.ROSInterruptException:
		pass




