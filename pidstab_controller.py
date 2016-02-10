#!/usr/bin/env python


#Julien LEIMER
#February 2016
#Implementation of a PID controller for stabilization around the upright position

from __future__ import division
import sys
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature
import serial
from pid import PID

class RobocapeController:
    def __init__(self):
        #initialize node
        rospy.init_node('RoboCapeController')
        rospy.loginfo(rospy.get_caller_id() + 'Initializing RoboCapeController node')

        #initialize serial communication (with remote control)
        self.dev = serial.Serial('/dev/ttyO2', 115200)

        #initialize global variables
        self.Roll = 0
        self.RollRate = 0
        self.ThrottleREMOTE = 0
        self.SteeringREMOTE = 0

        self.SteeringCONTROLdeg = 0
        self.SteeringCONTROLpulse = 0

        self.Kp = -0.654276982979989
        self.Ki = -2.28191654162741
        self.Kd = -0.0211479278661121
            
        #intialize PID controller
        self.pid = PID.PID(Kp, Ki, Kd)
        self.pid.SetPoint=0.0
        self.pid.setSampleTime(1/30)

    def subscribe(self):
        #subscribe to IMU messages
        rospy.Subscriber('imu_readings', Imu, self.callbackIMU)
        #subscribe to REMOTE messages
        #rospy.Subscriber('remote_readings', Temperature, self.callbackREMOTE)

    def callbackIMU(self, msg):
        #save IMU messages into global variables
        self.Roll = msg.orientation.x
        self.RollRate = msg.angular_velocity.x
        rospy.loginfo([self.Roll, self.RollRate])

    def callbackREMOTE(self, msg):
        #save REMOTE messages into global variables
        self.ThrottleREMOTE = msg.temperature
        self.SteeringREMOTE = msg.variance
        rospy.loginfo([self.ThrottleREMOTE, self.SteeringREMOTE])

    def publish(self):
        pub = rospy.Publisher('controller_readings', Temperature, queue_size=1)
        rate = rospy.Rate(40)
        msg_temp = Temperature()
        msg_temp.header.frame_id = "robocape"

        while not rospy.is_shutdown():
            now = rospy.get_rostime()
            
            
            #read remote commands
            while self.dev.inWaiting() <= 11:
                pass
            UART_read = self.dev.readline()
            self.dev.flushInput()
            self.SteeringREMOTE = int(UART_READ[0:4])
            self.ThrottleREMOTE = int(UART_READ[4:8])
            rospy.loginfo([self.ThrottleREMOTE, self.SteeringREMOTE])

            #controller
            self.pid.update(self.roll)
            self.SteeringCONTROLdeg = self.pid.output
            self.SteeringCONTROLpulse = ((self.SteeringControldeg + 66)*7.35294118)+1000

            #steering saturation
            if self.SteeringCONTROLpulse > 1800:
                msg_temp.variance = 1800
            elif self.SteeringCONTROLpulse < 1200:
                msg_temp.variance = 1200
            else:
                msg_temp.variance = self.SteeringCONTROLpulse

            #publish messages to the actuators
            msg_temp.header.stamp.secs = now.secs
            msg_temp.header.stamp.nsecs = now.nsecs
            msg_temp.temperature = self.ThrottleREMOTE
            pub.publish(msg_temp)
            rospy.loginfo("Publishing :")
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
    except rospy.ROSInterruptException:
        pass




