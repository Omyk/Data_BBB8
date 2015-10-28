from mpu9150_driver import Mpu9150
import time

imu = Mpu9150(0x69);
imu.calibrate();
(mag_offset, mag_scale) = imu.magCalibration();

while True:
	imu.update();
	imu.compass[0] -= mag_offset[0];
	imu.compass[1] += mag_offset[1];
	imu.compass[2] -= mag_offset[2];
	print imu.raw_data();
	time.sleep(0.2);

