#!/usr/bin/python
# -*- coding:utf-8 -*-
"""
This module is for IMU related functions

"""

from typing import Union
import time
from PiFinder import config
from PiFinder.multiproclogging import MultiprocLogging
import board
import adafruit_bno055
import logging
import quaternion  # Numpy quaternion

logger = logging.getLogger("IMU.pi")

QUEUE_LEN = 10


class Imu:
    """
    Previous version modified the IMU axes but the IMU now outputs the
    measurements using its native axes and the transformation from the IMU
    axes to the camera frame is done by the IMU dead-reckonig functionality.
    """
    MAX_SLEEP_TIME = 1.0  # [s] Max sleep time to avoid sleeping for too long
    N_GYRO_CAL_SAMPLES = 150

    def __init__(self):
        self.sensor = self.configure_imu_bno055()

        self.__moving = False

        # Sampling frequency
        self.imu_sample_frequency = 1 / 30
        self.imu_sample_period = 1 / self.imu_sample_frequency

        # IMU data
        self.timestamp = None
        self.gyro = None  # Gyroscope data in rad/s
        self.accel = None  # Accelerometer data in m/s^2
        self.quat = None  # Quaternion data quaternion.quaternion(w, x, y, z)
        self.outdated_data = True

        self.last_sample_time = time.time()  # Last time stamp when data was read from the IMU (but not necessarily valid and stored)

        # Calibration
        self.gyro_calibrated = False
        #self.accel_calibrated = False
        self.gyro_offsets = None

        # Internal states
        self.moving = False

        # First value is delta to exceed between samples to start moving,
        # second is threshold to fall below to stop moving.
        cfg = config.Config()
        #imu_threshold_scale = cfg.get_option("imu_threshold_scale", 1)
        self.moving_thresholds = (0.5, 0.3)

    def configure_imu_bno055(self):
        """
        Configure the IMU.

        Accelerometer:

        Accelerometer range options:
        ACCEL_2G, ACCEL_4G, ACCEL_8G, ACCEL_16G

        Bandwidth options:
        ACCEL_7_81HZ, ACCEL_15_63HZ, ACCEL_31_25HZ, ACCEL_62_5HZ,
        ACCEL_125HZ, ACCEL_250HZ, ACCEL_500HZ, ACCEL_1000HZ

        Gyro::

        Gyro range options:
        GYRO_2000_DPS, GYRO_1000_DPS, GYRO_500_DPS, GYRO_250_DPS, GYRO_125_DPS

        Gyro bandwidth options:
        GYRO_523HZ, GYRO_230HZ, GYRO_116HZ, GYRO_47HZ, GYRO_23HZ, GYRO_12HZ
        GYRO_64HZ, GYRO_32HZ
        """
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c)
        # ACCGYRO mode: Accelerometer + Gyro data only, no fusion
        sensor.mode = adafruit_bno055.ACCGYRO_MODE

        # Configure accelerometer: Range +/-4G, Bandwidth 7.81Hz
        sensor.accel_range = adafruit_bno055.ACCEL_4G
        sensor.accel_bandwidth = adafruit_bno055.ACCEL_7_81HZ

        # Configure gyro: Range +/-500 dps, Bandwidth 12Hz
        sensor.gyro_range = adafruit_bno055.GYRO_500_DPS
        sensor.gyro_bandwidth = adafruit_bno055.GYRO_12HZ

        return sensor

    def read_raw_data(self):
        """ 
        Reads in the data from the IMU and returns the raw, uncalibrated
        data.
        """
        # check for a new sample period
        if time.time() - self.last_sample_time < self.imu_sample_period:
            self.sleep_until_next_sample()

        # Read in the new sample
        self.outdated_data = True
        timestamp, accel, gyro = self.read_from_imu()
        self.last_sample_time = timestamp

        return timestamp, accel, gyro

    def update(self) -> bool:
        ''' 
        Reads in the quaternion from the IMU. Returns True if a new valid
        sample is available. 
        '''
        # Read in the new raw samples
        timestamp, accel, gyro = self.read_raw_data()
        if gyro is None or accel is None:
            return False  # Failed to get sensor values

        # Check calibration status. If not calibrated, start calibration
        if not self.gyro_calibrated:
            if not self.estimate_gyro_calibration():
                return False
        # NOTE: Ignoring accelerometer calibration for now

        # Apply calibration
        gyro = self.apply_gyro_calibration(gyro)

        # Check outliers
        # TODO

        # Determine if moving
        # TODO

        # Valid sample obtained
        self.timestamp = timestamp
        self.gyro = gyro  # Gyroscope data in rad/s
        self.accel = accel  # Accelerometer data in m/s^2
        self.outdated_data = False

        # Construct quaternion
        # TODO

        return True # New sample available

    def read_from_imu(self):
        timestamp = time.time()
        accel = np.array(self.sensor.acceleration)  # Acceleration in m/s^2
        gyro = np.array(self.sensor.gyro)  # Gyroscope in rad/s

        if accel[0] is None:
            logger.warning("IMU: Failed to get accelerometer values")
            accel = None
            
        if gyro[0] is None:
            logger.warning("IMU: Failed to get gyro values")
            gyro = None
        
        return timestamp, accel, gyro

    def estimate_gyro_calibration(self) -> bool:
        """ 
        Determine the gyroscope offset by averaging over multipe samples while
        the IMU is stationary.
        """
        # TODO
        self.gyro_offsets = [0.0, 0.0, 0.0]
        self.gyro_calibrated = True
        logger.info("Gyro calibrated")
        return True

    def apply_gyro_calibration(self, gyro: np.ndarray):
        if self.gyro_calibrated and gyro is not None:
            return gyro - self.gyro_offsets
        else:
            return None
        
    def sleep_until_next_sample(self):
        ''' Sleep for the remaining time in the sampling period '''
        sleep_time = self.imu_sample_period - (time.time() - self.last_sample_time)
        if sleep_time > 0:
            time.sleep(min(sleep_time, MAX_SLEEP_TIME))


    def __str__(self):
        return (
            f"IMU Information:\n"
            f"Calibration Status: {self.calibration}\n"
            f"Quaternion History: {self.quat_history}\n"
            f"Average Quaternion: {self.avg_quat}\n"
            f"Moving: {self.moving}\n"
            f"Reading Difference: {self.__reading_diff}\n"
            f"Flip Count: {self._flip_count}\n"
            f"Last Sample Time: {self.last_sample_time}\n"
            f"IMU Sample Frequency: {self.imu_sample_frequency}\n"
            f"Moving Threshold: {self.__moving_threshold}\n"
        )


class gyro_calibration:
    """ 
    Calibrate the gyro by storing N measurements while stationary and taking
    the average.
    """

    def __init__(self):
        pass

    def start(self):
        """
        Start the gyro calibration. Flush the buffer.
        """
        pass

    def buffer_measurement(self):
        pass

    def estimate_offsets(self):
        pass


def imu_monitor(shared_state, console_queue, log_queue):
    MultiprocLogging.configurer(log_queue)
    logger.debug("Starting IMU")
    imu = None
    try:
        imu = Imu()
    except Exception as e:
        logger.error(f"Error starting phyiscal IMU : {e}")
        logger.error("Falling back to fake IMU")
        console_queue.put("IMU: Error starting physical IMU, using fake IMU")
        console_queue.put("DEGRADED_OPS IMU")
        from PiFinder.imu_fake import Imu as ImuFake

        imu = ImuFake()

    imu = Imu()
    imu_calibrated = False
    # TODO: Remove move_start, move_end
    imu_data = {
        "moving": False,
        "move_start": None,
        "move_end": None,
        "quat": quaternion.quaternion(
            0, 0, 0, 0
        ),  # Scalar-first numpy quaternion(w, x, y, z) - Init to invalid quaternion
        "status": 0,  # IMU Status: 3=Calibrated
    }

    while True:
        imu.update()
        imu_data["status"] = imu.calibration

        # TODO: move_start and move_end don't seem to be used?
        if imu.moving:
            if not imu_data["moving"]:
                logger.debug("IMU: move start")
                imu_data["moving"] = True
                imu_data["move_start"] = time.time()
            # DISABLE old method
            imu_data["quat"] = quaternion.from_float_array(
                imu.avg_quat
            )  # Scalar-first (w, x, y, z)
        else:
            if imu_data["moving"]:
                # If we were moving and we now stopped
                logger.debug("IMU: move end")
                imu_data["moving"] = False
                imu_data["move_end"] = time.time()
                imu_data["quat"] = quaternion.from_float_array(
                    imu.avg_quat
                )  # Scalar-first (w, x, y, z)

        if not imu_calibrated:
            if imu_data["status"] == 3:
                imu_calibrated = True
                console_queue.put("IMU: NDOF Calibrated!")

        if shared_state is not None and imu_calibrated:
            shared_state.set_imu(imu_data)


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger.info("Trying to read state from IMU")
    imu = None
    try:
        imu = Imu()
        for i in range(10):
            imu.update()
            print(imu)
            time.sleep(0.5)
    except Exception as e:
        logger.exception("Error starting phyiscal IMU", e)
