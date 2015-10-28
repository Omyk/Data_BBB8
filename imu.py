class Imu(object):
    def __init__(self):
        self.accel = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.compass = [0.0, 0.0, 0.0]
        self.pressure = 0.0
        self.temp = 0.0
	self.mag_offset = [0.0, 0.0, 0.0];
	self.mag_scale = [0.0, 0.0, 0.0];

    def raw_data(self):
        'Return a string containing raw data readings'
        pass

    def update(self):
        'Update all sensors data'
        pass

    def calibrate(self):
	'Calibrate biases'
	pass

    def magCalibration(self):
	'Return magnetometer offsets and scaling error'
	pass
