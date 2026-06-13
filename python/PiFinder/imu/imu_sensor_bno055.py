""" 
Configures the BNO055 IMU sensor 
"""
import board
import adafruit_bno055
import logging
from dataclasses import dataclass
import numpy as np

from imu_sensor import ImuSensor, GyroSpecs

logger = logging.getLogger("imu_bno055")


class ImuSensorBno055(ImuSensor):

    def __init__(self, imu_mode: str = "gyro"):
        self.configure(imu_mode)

    def configure(imu_mode: str):
        logger.info("IMU: Starting BNO055 IMU")
        self._reset_configurations()
        i2c = board.I2C()
        slef.sensor = adafruit_bno055.BNO055_I2C(i2c)

        # Set the IMU mode, sensor range and bandwidth
        if imu_mode == "accgyro":
            self._configure_accgyro_mode()
        elif imu_mode == "gyro":
            self._configure_gyro_mode()
        elif imu_mode == "fusion":
            raise NotImplementedError
        else:
            raise ValueError(f"Unknown IMU mode: {imu_mode}")
        
        self.imu_mode = imu_mode
        self.gyro_specs = GyroSpecsBno055(bandwidth=self.gyro_bandwidth)

    # ------ Configures the IMU mode ----------

    def _configure_accgyro_mode(self):
        """
        Configure in the accelerometer + gyro mode (no fusion)
        """
        self.sensor.mode = adafruit_bno055.ACCGYRO_MODE

        self._set_accelerometer_range()
        self._set_accelerometer_bandwidth()
        self.accelerometer_enabled = True
        
        self._set_gyro_range()
        self._set_gyro_bandwidth()
        self.gyro_enabled = True

        logger.info("IMU: Configured in ACCGYRO mode")

    def _configure_gyro_mode(self):
        """
        Configure the gyro-only mode
        """
        self.sensor.mode = adafruit_bno055.GYRONLY_MODE

        self._set_gyro_range()
        self._set_gyro_bandwidth()
        self.gyro_enabled = True

        logger.info("IMU: Configured in GYRONLY mode")

    # -------- Set the range and data bandwidth of each sensor ------

    def _set_gyro_range(self):
        """
        Configure the gyro sensitivity (range).

        Gyro range options:
        GYRO_2000_DPS, GYRO_1000_DPS, GYRO_500_DPS, GYRO_250_DPS, GYRO_125_DPS
        """
        # Configure gyro: Range +/-500 dps
        self.sensor.gyro_range = adafruit_bno055.GYRO_500_DPS
        logger.info("IMU: Cofigured gyro range")

    def _set_gyro_bandwidth(self):
        """
        Configure the gyro bandwidth.

        Gyro bandwidth options:
        GYRO_523HZ, GYRO_230HZ, GYRO_116HZ, GYRO_47HZ, GYRO_23HZ, GYRO_12HZ
        GYRO_64HZ, GYRO_32HZ
        """
        # Configure gyro: Bandwidth 12Hz
        self.sensor.gyro_bandwidth = adafruit_bno055.GYRO_12HZ
        self.gyro_bandwidth = 12
        logger.info(f"IMU: Configured bandwidth: {self.gyro_bandwidth} Hz")

    def _set_accelerometer_range(self):
        """
        Configure the accelerometer sensitivity (range).

        Accelerometer range options:
        ACCEL_2G, ACCEL_4G, ACCEL_8G, ACCEL_16G
        """
        # Configure accelerometer: Range +/-4G
        self.sensor.accel_range = adafruit_bno055.ACCEL_4G
        logger.info("IMU: Cofigured accelerometer range")

    def _set_accelerometer_bandwidth(self):
        """
        Configure the accelerometer bandwidth.

        Bandwidth options:
        ACCEL_7_81HZ, ACCEL_15_63HZ, ACCEL_31_25HZ, ACCEL_62_5HZ,
        ACCEL_125HZ, ACCEL_250HZ, ACCEL_500HZ, ACCEL_1000HZ
        """
        # Configure accelerometer: Bandwidth 7.81Hz
        self.sensor.accel_bandwidth = adafruit_bno055.ACCEL_7_81HZ
        self.accelerometer_bandwidth = 7.81
        logger.info("IMU: Cofigured accelerometer bandwidth")


@dataclass
class GyroSpecsBno055(GyroSpecs):
    """ From the BNO055 data sheet  """
    zero_rate_offset = np.deg2rad(3.0)  # Max zero rate offset [rad/s]
    output_noise_density = np.deg2rad(0.042)  # Max output noise density [rad/s/sqrt(Hz)]
