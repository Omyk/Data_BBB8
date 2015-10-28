from mpu9150_driver import Mpu9150
from magdwick import MadgwickAHRS
import time

imu = Mpu9150(0x69);
#imu.calibrate();
#(mag_offset, mag_scale) = imu.magCalibration();

madgwick= MadgwickAHRS(0.02, 2.7125, 0.0);
SEq = [1.0, 0.0, 0.0, 0.0]
b = [1.0, 0.0, 0.0]
wb = [0.0, 0.0, 0.0]
Euler = [0.0, 0.0, 0.0]
Euler2 = [0.0, 0.0, 0.0]

accel = [0.0, 0.0, 0.0]
compass = [0.0, 0.0, 0.0]

while True:
	begin = time.time()
	imu.update();
	#imu.compass[0] -= mag_offset[0];
	#imu.compass[1] += mag_offset[1];
	#imu.compass[2] -= mag_offset[2];
	accel[0] = imu.accel[0]/9.81
	accel[1] = imu.accel[1]/9.81
	accel[2] = imu.accel[2]/9.81
	compass[0] = imu.compass[0]/100
	compass[1] = imu.compass[1]/100
	compass[2] = imu.compass[2]/100

	(SEq, b, wb) = madgwick.filterUpdate(imu.gyro, accel, compass, SEq, b, wb)	
	Euler = madgwick.quatern2euler(madgwick.quaternConj(SEq))
	Euler[0] *= (180/3.14159265358979)
	Euler[1] *= (180/3.14159265358979)
	Euler[2] *= (180/3.14159265358979)
	print Euler
	Euler2 = madgwick.EulerUpdateFilter(imu.gyro, imu.accel, imu.compass)
	print Euler2
	print imu.raw_data();
	time.sleep(0.02);
	length = time.time() - begin
	print length

