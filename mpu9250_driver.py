from __future__ import division
from imu import Imu
import mraa
import math

### MPU 9150 Register Map
MPU9150_SELF_TEST_X        = 0x0D   # R/W
MPU9150_SELF_TEST_Y        = 0x0E   # R/W
MPU9150_SELF_TEST_X        = 0x0F   # R/W
MPU9150_SELF_TEST_A        = 0x10   # R/W
MPU9150_SMPLRT_DIV         = 0x19   # R/W
MPU9150_CONFIG             = 0x1A   # R/W
MPU9150_GYRO_CONFIG        = 0x1B   # R/W
MPU9150_ACCEL_CONFIG       = 0x1C   # R/W
MPU9150_FF_THR             = 0x1D   # R/W
MPU9150_FF_DUR             = 0x1E   # R/W
MPU9150_MOT_THR            = 0x1F   # R/W
MPU9150_MOT_DUR            = 0x20   # R/W
MPU9150_ZRMOT_THR          = 0x21   # R/W
MPU9150_ZRMOT_DUR          = 0x22   # R/W
MPU9150_FIFO_EN            = 0x23   # R/W
MPU9150_I2C_MST_CTRL       = 0x24   # R/W
MPU9150_I2C_SLV0_ADDR      = 0x25   # R/W
MPU9150_I2C_SLV0_REG       = 0x26   # R/W
MPU9150_I2C_SLV0_CTRL      = 0x27   # R/W
MPU9150_I2C_SLV1_ADDR      = 0x28   # R/W
MPU9150_I2C_SLV1_REG       = 0x29   # R/W
MPU9150_I2C_SLV1_CTRL      = 0x2A   # R/W
MPU9150_I2C_SLV2_ADDR      = 0x2B   # R/W
MPU9150_I2C_SLV2_REG       = 0x2C   # R/W
MPU9150_I2C_SLV2_CTRL      = 0x2D   # R/W
MPU9150_I2C_SLV3_ADDR      = 0x2E   # R/W
MPU9150_I2C_SLV3_REG       = 0x2F   # R/W
MPU9150_I2C_SLV3_CTRL      = 0x30   # R/W
MPU9150_I2C_SLV4_ADDR      = 0x31   # R/W
MPU9150_I2C_SLV4_REG       = 0x32   # R/W
MPU9150_I2C_SLV4_DO        = 0x33   # R/W
MPU9150_I2C_SLV4_CTRL      = 0x34   # R/W
MPU9150_I2C_SLV4_DI        = 0x35   # R
MPU9150_I2C_MST_STATUS     = 0x36   # R
MPU9150_INT_PIN_CFG        = 0x37   # R/W
MPU9150_INT_ENABLE         = 0x38   # R/W
MPU9150_INT_STATUS         = 0x3A   # R
MPU9150_ACCEL_XOUT_H       = 0x3B   # R
MPU9150_ACCEL_XOUT_L       = 0x3C   # R
MPU9150_ACCEL_YOUT_H       = 0x3D   # R
MPU9150_ACCEL_YOUT_L       = 0x3E   # R
MPU9150_ACCEL_ZOUT_H       = 0x3F   # R
MPU9150_ACCEL_ZOUT_L       = 0x40   # R
MPU9150_TEMP_OUT_H         = 0x41   # R
MPU9150_TEMP_OUT_L         = 0x42   # R
MPU9150_GYRO_XOUT_H        = 0x43   # R
MPU9150_GYRO_XOUT_L        = 0x44   # R
MPU9150_GYRO_YOUT_H        = 0x45   # R
MPU9150_GYRO_YOUT_L        = 0x46   # R
MPU9150_GYRO_ZOUT_H        = 0x47   # R
MPU9150_GYRO_ZOUT_L        = 0x48   # R
MPU9150_EXT_SENS_DATA_00   = 0x49   # R
MPU9150_EXT_SENS_DATA_01   = 0x4A   # R
MPU9150_EXT_SENS_DATA_02   = 0x4B   # R
MPU9150_EXT_SENS_DATA_03   = 0x4C   # R
MPU9150_EXT_SENS_DATA_04   = 0x4D   # R
MPU9150_EXT_SENS_DATA_05   = 0x4E   # R
MPU9150_EXT_SENS_DATA_06   = 0x4F   # R
MPU9150_EXT_SENS_DATA_07   = 0x50   # R
MPU9150_EXT_SENS_DATA_08   = 0x51   # R
MPU9150_EXT_SENS_DATA_09   = 0x52   # R
MPU9150_EXT_SENS_DATA_10   = 0x53   # R
MPU9150_EXT_SENS_DATA_11   = 0x54   # R
MPU9150_EXT_SENS_DATA_12   = 0x55   # R
MPU9150_EXT_SENS_DATA_13   = 0x56   # R
MPU9150_EXT_SENS_DATA_14   = 0x57   # R
MPU9150_EXT_SENS_DATA_15   = 0x58   # R
MPU9150_EXT_SENS_DATA_16   = 0x59   # R
MPU9150_EXT_SENS_DATA_17   = 0x5A   # R
MPU9150_EXT_SENS_DATA_18   = 0x5B   # R
MPU9150_EXT_SENS_DATA_19   = 0x5C   # R
MPU9150_EXT_SENS_DATA_20   = 0x5D   # R
MPU9150_EXT_SENS_DATA_21   = 0x5E   # R
MPU9150_EXT_SENS_DATA_22   = 0x5F   # R
MPU9150_EXT_SENS_DATA_23   = 0x60   # R
MPU9150_MOT_DETECT_STATUS  = 0x61   # R
MPU9150_I2C_SLV0_DO        = 0x63   # R/W
MPU9150_I2C_SLV1_DO        = 0x64   # R/W
MPU9150_I2C_SLV2_DO        = 0x65   # R/W
MPU9150_I2C_SLV3_DO        = 0x66   # R/W
MPU9150_I2C_MST_DELAY_CTRL = 0x67   # R/W
MPU9150_SIGNAL_PATH_RESET  = 0x68   # R/W
MPU9150_MOT_DETECT_CTRL    = 0x69   # R/W
MPU9150_USER_CTRL          = 0x6A   # R/W
MPU9150_PWR_MGMT_1         = 0x6B   # R/W
MPU9150_PWR_MGMT_2         = 0x6C   # R/W
MPU9150_FIFO_COUNTH        = 0x72   # R/W
MPU9150_FIFO_COUNTL        = 0x73   # R/W
MPU9150_FIFO_R_W           = 0x74   # R/W
MPU9150_WHO_AM_I           = 0x75   # R

# MPU9150 Compass
MPU9150_CMPS_XOUT_L        = 0x4A   # R
MPU9150_CMPS_XOUT_H        = 0x4B   # R
MPU9150_CMPS_YOUT_L        = 0x4C   # R
MPU9150_CMPS_YOUT_H        = 0x4D   # R
MPU9150_CMPS_ZOUT_L        = 0x4E   # R
MPU9150_CMPS_ZOUT_H        = 0x4F   # R


class Mpu9250(Imu):
    def __init__(self, gyro_scale=250.0, accel_scale=2.0):
        super(Mpu9250, self).__init__()

        self.gyro_scale = gyro_scale   # 250, 500, 1000 or 2000 deg/s
        self.accel_scale = accel_scale # 2, 4, 8 or 16 g
        self.compass_scale = 1200.0    # 1200 uT

        self.dev = mraa.Spi(0)
	self.dev.mode(mraa.SPI_MODE3)
	self.dev.frequency(800000)
	self.dev.bitPerWord(16)

        self.__setup_conf()

        # Compute scaling factors
        self.gyro_scaling = (65536.0 / (2 * gyro_scale)) * (180.0 / math.pi)
        self.accel_scaling = 65536.0 / (2 * accel_scale * 9.81)
        self.compass_scaling = 4000.0 / self.compass_scale

    def raw_data(self):
        info = 'Acceleration vector (in m/s^2):\n'
        info += 'x: {:.3f}, y: {:.3f}, z: {:.3f}\n'.format(self.accel[0], self.accel[1], self.accel[2])
        info += 'Angular velocity vector (in rad/s):\n'
        info += 'x: {:.2f}, y: {:.2f}, z: {:.2f}\n'.format(self.gyro[0], self.gyro[1], self.gyro[2])
        info += 'Compass heading vector (in uT):\n'
        info += 'x: {:.2f}, y: {:.2f}, z: {:.2f}\n'.format(self.compass[0], self.compass[1], self.compass[2])
        info += 'Temperature (in C): {:.2f}\n'.format(self.temp)
	return info;

    def update(self):
        self.accel[0] = self.__read_word(MPU9150_ACCEL_XOUT_H, MPU9150_ACCEL_XOUT_L) / self.accel_scaling
        self.accel[1] = self.__read_word(MPU9150_ACCEL_YOUT_H, MPU9150_ACCEL_YOUT_L) / self.accel_scaling
        self.accel[2] = self.__read_word(MPU9150_ACCEL_ZOUT_H, MPU9150_ACCEL_ZOUT_L) / self.accel_scaling

        self.gyro[0] = self.__read_word(MPU9150_GYRO_XOUT_H, MPU9150_GYRO_XOUT_L) / self.gyro_scaling
        self.gyro[1] = self.__read_word(MPU9150_GYRO_YOUT_H, MPU9150_GYRO_YOUT_L) / self.gyro_scaling
        self.gyro[2] = self.__read_word(MPU9150_GYRO_ZOUT_H, MPU9150_GYRO_ZOUT_L) / self.gyro_scaling

        self.compass[0] = self.__read_word(MPU9150_CMPS_XOUT_H, MPU9150_CMPS_XOUT_L) / self.compass_scaling
        self.compass[1] = self.__read_word(MPU9150_CMPS_YOUT_H, MPU9150_CMPS_YOUT_L) / self.compass_scaling
        self.compass[2] = self.__read_word(MPU9150_CMPS_ZOUT_H, MPU9150_CMPS_ZOUT_L) / self.compass_scaling

        self.temp = self.__read_word(MPU9150_TEMP_OUT_H, MPU9150_TEMP_OUT_L) / 340.0 + 35

    def __setup_conf(self):
        'Runs boot up configuration to wake up IMU and sets options as wanted'
        # Turn ON
        self.__writeReg(MPU9150_PWR_MGMT_1, 0x01)  # Power up
        self.__writeReg(MPU9150_INT_PIN_CFG, 0x00)
        self.__writeReg(MPU9150_USER_CTRL, 0x00)

        # Configure Gyroscope and Accelerometer
        self.__writeReg(MPU9150_GYRO_CONFIG, 0x00)
        self.__writeReg(MPU9150_ACCEL_CONFIG, 0x00)
        self.__writeReg(MPU9150_SMPLRT_DIV, 0x0A)  # Set sample rate at 100Hz
        # Setup low pass filter
        self.__writeReg(MPU9150_CONFIG, 0x00)
        self.__writeReg(MPU9150_CONFIG, 0x05)  # Set lowpass response at 10Hz
        # Set Gyroscope update rate at 1kHz (same as accelerometer)
        self.__writeReg(MPU9150_SMPLRT_DIV, 0x07)

        # Boot compass & configure it
        self.__writeReg(0x0A,0x00)
        self.__writeReg(0x0A,0x0F) # Self-test
        self.__writeReg(0x0A,0x00)

        # Configure i2c communication/reading
        self.__writeReg(MPU9150_I2C_MST_CTRL,0x40)  # Wait for Data at slave0
        self.__writeReg(MPU9150_I2C_SLV0_ADDR,0x8C) # Set slave0 i2c addr at 0x0C
        self.__writeReg(MPU9150_I2C_SLV0_REG,0x02)  # Set reading start at slave0
        self.__writeReg(MPU9150_I2C_SLV0_CTRL,0x88) # Enable & set offset
        self.__writeReg(MPU9150_I2C_SLV1_ADDR,0x0C) # Set slave1 i2c addr at 0x0C
        self.__writeReg(MPU9150_I2C_SLV1_REG,0x0A)  # Set reading start at slave1
        self.__writeReg(MPU9150_I2C_SLV1_CTRL,0x81) # Enable at set length to 1
        self.__writeReg(MPU9150_I2C_SLV1_DO,0x01)
        self.__writeReg(MPU9150_I2C_MST_DELAY_CTRL,0x03) # Set delay rate
        self.__writeReg(0x01,0x80)                   # Set i2c slave4 delay
        self.__writeReg(MPU9150_I2C_SLV4_CTRL,0x04)
        self.__writeReg(MPU9150_I2C_SLV1_DO,0x00)    # Clear user settings
        self.__writeReg(MPU9150_USER_CTRL,0x00)
        self.__writeReg(MPU9150_I2C_SLV1_DO,0x01)    # Override register
        self.__writeReg(MPU9150_USER_CTRL,0x20)      # Enable master i2c mode
        self.__writeReg(MPU9150_I2C_SLV4_CTRL,0x13)  # Disable slv4

        # Set Gyroscope scale
        if self.gyro_scale <= 250.0:
            self.gyro_scale = 250
            self.__writeReg(MPU9150_GYRO_CONFIG, 0x00)
        elif self.gyro_scale <= 500.0:
            self.gyro_scale = 500
            self.__writeReg(MPU9150_GYRO_CONFIG, 0x08)
        elif self.gyro_scale <= 1000.0:
            self.gyro_scale = 1000
            self.__writeReg(MPU9150_GYRO_CONFIG, 0x10)
        else:
            self.gyro_scale = 2000
            self.__writeReg(MPU9150_GYRO_CONFIG, 0x18)

        # Set Accelerometer scale
        if self.accel_scale <= 2:
            self.accel_scale = 2
            self.__writeReg(MPU9150_ACCEL_CONFIG, 0x00)
        elif self.accel_scale <= 4:
            self.accel_scale = 4
            self.__writeReg(MPU9150_ACCEL_CONFIG, 0x08)
        elif self.accel_scale <= 8:
            self.accel_scale = 8
            self.__writeReg(MPU9150_ACCEL_CONFIG, 0x10)
        else:
            self.accel_scale = 16
            self.__writeReg(MPU9150_ACCEL_CONFIG, 0x18)

    def __read_word(self, reg_h, reg_l):
        'Reads data from high & low registers and returns the combination'
        # Read register values
        val_h = self.__readReg(reg_h)
        val_l = self.__readReg(reg_l)

        if val_h > 127:
            return (((val_h - 256) << 8) - val_l)
        else:
            return ((val_h << 8) + val_l)

    def __writeReg(self, address, value):
        self.dev.write_word(((address & 0x7F) << 8) | (value & 0xFF));
        return;

    def __readReg(self, address):
        data = self.dev.write_word(((address & 0x7F) << 8) | 0x8000);
        return data;

