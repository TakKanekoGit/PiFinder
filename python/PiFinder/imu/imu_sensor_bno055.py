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

    sensor: adafruit_bno055.BNO055_I2C

    def __init__(self, imu_mode: str = "gyro"):
        self.configure(imu_mode)

    def configure(imu_mode: str):
        logger.info("IMU: Starting BNO055 IMU")
        i2c = board.I2C()
        slef.sensor = adafruit_bno055.BNO055_I2C(i2c)

        if imu_mode == "accgyro":
            self._configure_accgyro_mode()
            self._set_accelerometer_range()
            self._set_accelerometer_bandwidth()
            self._set_gyro_range()
            self._set_gyro_bandwidth()
        elif imu_mode == "gyro":
            self._configure_gyro_mode()
            self._set_gyro_range()
            self._set_gyro_bandwidth()
        elif imu_mode == "fusion":
            raise NotImplementedError
        else:
            raise ValueError(f"Unknown IMU mode: {imu_mode}")

    # ------ Configures the IMU mode ----------

    def _configure_accgyro_mode(self):
        """
        Configure in the accelerometer + gyro mode
        """
        # ACCGYRO mode: Accelerometer + Gyro data only, no fusion
        self.sensor.mode = adafruit_bno055.ACCGYRO_MODE
        logger.info("IMU: Configured in ACCGYRO mode")

    def _configure_gyro_mode(self):
        """
        Configure the gyro-only mode
        """
        # ACCGYRO mode: Accelerometer + Gyro data only, no fusion
        self.sensor.mode = adafruit_bno055.GYRONLY_MODE
        logger.info("IMU: Configured in GYRONLY mode")

    # -------- Set the range and data bandwidth of each sensor ------

    def _set_gyro_range(self):
        """
        Configure the gyro sensitivity (range).

        Gyro range options:
        GYRO_2000_DPS, GYRO_1000_DPS, GYRO_500_DPS, GYRO_250_DPS, GYRO_125_DPS
        """
        # Configure gyro: Range +/-500 dps, Bandwidth 12Hz
        self.sensor.gyro_range = adafruit_bno055.GYRO_500_DPS
        logger.info("IMU: Cofigured gyro range")

    def _set_gyro_bandwidth(self):
        """
        Configure the gyro bandwidth.

        Gyro bandwidth options:
        GYRO_523HZ, GYRO_230HZ, GYRO_116HZ, GYRO_47HZ, GYRO_23HZ, GYRO_12HZ
        GYRO_64HZ, GYRO_32HZ
        """
        # Configure gyro: Range +/-500 dps, Bandwidth 12Hz
        self.sensor.gyro_bandwidth = adafruit_bno055.GYRO_12HZ
        logger.info("IMU: Configured bandwidth")

    def _set_accelerometer_range(self):
        """
        Configure the accelerometer sensitivity (range).

        Accelerometer range options:
        ACCEL_2G, ACCEL_4G, ACCEL_8G, ACCEL_16G
        """
        # Configure accelerometer: Range +/-4G, Bandwidth 7.81Hz
        self.sensor.accel_range = adafruit_bno055.ACCEL_4G
        logger.info("IMU: Cofigured accelerometer range")

    def _set_accelerometer_bandwidth(self):
        """
        Configure the accelerometer bandwidth.

        Bandwidth options:
        ACCEL_7_81HZ, ACCEL_15_63HZ, ACCEL_31_25HZ, ACCEL_62_5HZ,
        ACCEL_125HZ, ACCEL_250HZ, ACCEL_500HZ, ACCEL_1000HZ
        """
        # Configure accelerometer: Range +/-4G, Bandwidth 7.81Hz
        self.sensor.accel_bandwidth = adafruit_bno055.ACCEL_7_81HZ
        logger.info("IMU: Cofigured accelerometer bandwidth")


@dataclass
class GyroSpecsBno055(GyroSpecs):
    """ From the BNO055 data sheet  """
    zero_rate_offset = np.deg2rad(3.0)  # Max zero rate offset [rad/s]
    output_noise_density = np.deg2rad(0.042)  # Max output noise density [rad/s/sqrt(Hz)]
    
    # Configured:
    #bandwidth = 12.0  # Bandwidth [Hz]
