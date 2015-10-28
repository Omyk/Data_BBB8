#!/usr/bin/env python

from mpu9250_driver import Mpu9250
import time

imu = Mpu9250();

while True:
	time.sleep(0.5);
	imu.update();
	print imu.raw_data();
