#!/usr/bin/env python
from __future__ import division
import rospy
import tf
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from mpu9150_driver import Mpu9150
#from mpu9250_driver import Mpu9250
from magdwick import MadgwickAHRS

class ImuHandler:
    def __init__(self):
        node_name = 'imu_mpu9150_handler'
        rospy.init_node(node_name)
        rospy.loginfo(rospy.get_caller_id() + 'Initialising {} node'.format(node_name))

        # Get all parameters from config (rosparam)
        address = int(rospy.get_param('imu/address', 0x69))
        gyro_scale = int(rospy.get_param('imu/gyroscope/scale', 250))
        acc_scale = int(rospy.get_param('imu/accelerometer/scale', 2))
	
	# Initialize Madgwick Algorithm
	self.madgwick = MadgwickAHRS(1/50, 5, 0.2);

	rospy.loginfo('Test {} address'.format(address))
        self.imu = Mpu9150(address, gyro_scale, acc_scale)
	#self.imu = Mpu9250(gyro_scale, acc_scale)
	self.imu.calibrate();
	rospy.loginfo("Compass calibration; move the IMU in random positions during the calibration");
	(self.mag_offset, self.mag_scale) = self.imu.magCalibration();
	rospy.loginfo("Calibration done");

    def publish(self):
        queue_size = rospy.get_param('imu/queue', 1);
        self.pub_imu = rospy.Publisher('imu_readings', Imu, queue_size=queue_size);
	self.pub_mag = rospy.Publisher('mag_readings', MagneticField, queue_size=queue_size);
        #self.rate = rospy.Rate(rospy.get_param('imu/rate', 10))
	#self.rate = rospy.Rate(20)
	self.rate = rospy.Rate(50)	

        (self.msg_imu, self.msg_mag) = self.msg_template()

        while not rospy.is_shutdown():
            #rospy.loginfo(rospy.get_caller_id() + "publishing IMU data")

            self.imu.update()
            self.pack_message()
            self.pub_imu.publish(self.msg_imu)
	    self.pub_mag.publish(self.msg_mag)

            self.rate.sleep()

    def msg_template(self):
        msg_imu = Imu()
	msg_mag = MagneticField();
        msg_imu.header.frame_id = "robocape"
	msg_mag.header.frame_id = "robocape_mag"

        msg_imu.linear_acceleration_covariance = (0.1, 0.0, 0.0,
                                              0.0, 0.1, 0.0,
                                              0.0, 0.0, 0.1)

        msg_imu.angular_velocity_covariance = (1.0, 0.0, 0.0,
                                           0.0, 1.0, 0.0,
                                           0.0, 0.0, 1.0)

        msg_imu.orientation_covariance = (10.0, 0.0,  0.0,
                                      0.0,  10.0, 0.0,
                                      0.0,  0.0,  10.0)

	msg_mag.magnetic_field_covariance = (10.0, 0.0, 0.0,
					0.0, 10.0, 0.0,
					0.0, 0.0, 10.0)

        return (msg_imu, msg_mag)

    def pack_message(self):
	Euler = [0.0, 0.0, 0.0]
        now = rospy.get_rostime()
        self.msg_imu.header.stamp.secs = now.secs
        self.msg_imu.header.stamp.nsecs = now.nsecs
	self.msg_mag.header.stamp.secs = now.secs
	self.msg_mag.header.stamp.nsecs = now.nsecs

        self.msg_imu.linear_acceleration.x = self.imu.accel[0]
        self.msg_imu.linear_acceleration.y = self.imu.accel[1]
        self.msg_imu.linear_acceleration.z = self.imu.accel[2]

        self.msg_imu.angular_velocity.x = self.imu.gyro[0]
        self.msg_imu.angular_velocity.y = self.imu.gyro[1]
        self.msg_imu.angular_velocity.z = self.imu.gyro[2]

	Euler = self.madgwick.EulerUpdateFilter(self.imu.gyro, self.imu.accel, self.imu.compass)
	self.msg_imu.orientation.x = Euler[0];
	self.msg_imu.orientation.y = Euler[1];
	self.msg_imu.orientation.z = Euler[2];
	self.msg_imu.orientation.w = 0;

	self.msg_mag.magnetic_field.x = self.imu.compass[0] - self.mag_offset[0];
	self.msg_mag.magnetic_field.y = self.imu.compass[1] + self.mag_offset[1];
	self.msg_mag.magnetic_field.z = self.imu.compass[2] - self.mag_offset[2];

        #self.msg_mag.magnetic_field.x = self.imu.compass[0]
        #self.msg_mag.magnetic_field.y = self.imu.compass[1]
        #self.msg_mag.magnetic_field.z = self.imu.compass[2]

        #quaternion = tf.transformations.quaternion_from_euler(self.imu.compass[0],
        #                                                      self.imu.compass[1],
        #                                                      self.imu.compass[2])
        #self.msg.orientation.x = quaternion[0]
        #self.msg.orientation.y = quaternion[1]
        #self.msg.orientation.z = quaternion[2]
        #self.msg.orientation.w = quaternion[3]


if __name__ == '__main__':
    try:
        imu = ImuHandler()
        imu.publish()
    except rospy.ROSInterruptException:
        pass
