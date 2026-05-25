

import board
import adafruit_bno055
import logging

logger = logging.getLogger("imu_bno055")


class ImuBno055:

    def __init__(self, enable_accel: bool):
        logger.info("IMU: Starting BNO055 IMU")
        if enable_accel:
            sensor = self.configure_imu_accgyro_mode()
        else:
            sensor = self._configure_imu_gyro_mode()


    @staticmethod
    def _configure_imu_accgyro_mode() -> adafruit_bno055.BNO055_I2C:
        """
        Configure the IMU.
        """
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c)

        # ACCGYRO mode: Accelerometer + Gyro data only, no fusion
        sensor.mode = adafruit_bno055.ACCGYRO_MODE
        sensor = self.configure_gyro(sensor)
        sensor = self.configure_accelerometer(sensor)

        logger.info("IMU: Configured in ACCGYRO mode")
        return sensor

    @staticmethod
    def _configure_imu_gyro_mode() -> adafruit_bno055.BNO055_I2C:
        """
        Configure the IMU.
        """
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c)

        # ACCGYRO mode: Accelerometer + Gyro data only, no fusion
        sensor.mode = adafruit_bno055.GYRONLY_MODE
        sensor = self.configure_gyro(sensor)

        logger.info("IMU: Configured in GYRONLY mode")
        return sensor

    @staticmethod
    def _configure_gyro(sensor: adafruit_bno055.BNO055_I2C) -> adafruit_bno055.BNO055_I2C:
        """
        Configure the gyro sensitivity (range) and bandwidth.

        Gyro range options:
        GYRO_2000_DPS, GYRO_1000_DPS, GYRO_500_DPS, GYRO_250_DPS, GYRO_125_DPS

        Gyro bandwidth options:
        GYRO_523HZ, GYRO_230HZ, GYRO_116HZ, GYRO_47HZ, GYRO_23HZ, GYRO_12HZ
        GYRO_64HZ, GYRO_32HZ
        """
        # Configure accelerometer: Range +/-4G, Bandwidth 7.81Hz
        sensor.accel_range = adafruit_bno055.ACCEL_4G
        sensor.accel_bandwidth = adafruit_bno055.ACCEL_7_81HZ
        logger.info("IMU: Cofigured accelerometer range and bandwidth")
        return sensor
    
    @staticmethod
    def _configure_accelerometer(sensor: adafruit_bno055.BNO055_I2C) -> adafruit_bno055.BNO055_I2C:
        """
        Configure the accelerometer sensitivity (range) and bandwidth.

        Accelerometer range options:
        ACCEL_2G, ACCEL_4G, ACCEL_8G, ACCEL_16G

        Bandwidth options:
        ACCEL_7_81HZ, ACCEL_15_63HZ, ACCEL_31_25HZ, ACCEL_62_5HZ,
        ACCEL_125HZ, ACCEL_250HZ, ACCEL_500HZ, ACCEL_1000HZ
        """
        # Configure gyro: Range +/-500 dps, Bandwidth 12Hz
        sensor.gyro_range = adafruit_bno055.GYRO_500_DPS
        sensor.gyro_bandwidth = adafruit_bno055.GYRO_12HZ
        logger.info("IMU: Cofigured gyro range and bandwidth")
        return sensor
