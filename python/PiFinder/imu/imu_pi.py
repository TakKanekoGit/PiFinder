#!/usr/bin/python
# -*- coding:utf-8 -*-
"""
This module is for IMU related functions

"""
from dataclasses import dataclass
import logging
import quaternion  # Numpy quaternion
import numpy as np
from typing import Union
import time

from PiFinder import config
from PiFinder.multiproclogging import MultiprocLogging

logger = logging.getLogger("IMU.pi")

# Type hint aliases
NdarrayNone = Union[np.ndarray, None]
QuaternionNone = Union[quaternion.quaternion, None]

QUEUE_LEN = 10  # TODO: Remove?


class Imu:
    """
    Previous version modified the IMU axes but the IMU now outputs the
    measurements using its native axes and the transformation from the IMU
    axes to the camera frame is done by the IMU dead-reckonig functionality.
    
    NOTE: Not all accelerometer features available but included for development
    """
    MAX_SLEEP_TIME = 1.0  # [s] Max sleep time to avoid sleeping for too long
    N_GYRO_CAL_SAMPLES = 150

    #sensor: adafruit_bno055.BNO055_I2C


    def __init__(self, enable_accel: bool = False, emulate: bool = False):
        self.sensor = self._init_sensor(enable_accel=enable_accel, emulate=emulate)
        self.accel_enabled = enable_accel

        # Sampling frequency
        self.imu_sample_frequency = 1 / 30  # Should be > 2 * sensor bandwidth
        self.imu_sample_period = 1 / self.imu_sample_frequency

        # IMU data
        self.imu_data = ImuRawData()

        self.last_read_time = time.time()  # Last time stamp when data was read from the IMU (but not necessarily valid and stored)

        # Calibration
        self.gyro_calibrated = False
        self.accel_calibrated = False
        self.gyro_offsets = None

        # Internal states
        self.moving = False
        self.__moving = False
        self._prev_quat = None
        self._prev_data_timestamp = time.time()

        # First value is delta to exceed between samples to start moving,
        # second is threshold to fall below to stop moving.
        cfg = config.Config()
        #imu_threshold_scale = cfg.get_option("imu_threshold_scale", 1)
        self.moving_thresholds = (0.5, 0.3)

    def _init_sensor(self, enable_accel: bool = False, emulate: bool = False):
        """
        Initializes the IMU sensor or the emulator.

        NOTE: Delay importing used to prevent issues when running in emulation 
        mode on non-Raspberry Pi HW.
        """
        if emulate:
            from PiFinder.imu.imu_sensor_emulator import configure_imu_sensor_emulator
            sensor = configure_imu_sensor_emulator(enable_accel)
        else:
            from PiFinder.imu.imu_sensor_bno055 import configure_imu_sensor_bno055
            sensor = configure_imu_sensor_bno055(enable_accel)

        return sensor

    def _read_raw_data(self) -> tuple[float, NdarrayNone, NdarrayNone]:
        """
        Reads in the data from the IMU and returns the raw, uncalibrated
        data.
        """
        # check for a new sample period
        if time.time() - self.last_read_time < self.imu_sample_period:
            self._sleep_until_next_sample()

        # Read in the new sample
        if self.accel_enabled:
            timestamp, accel, gyro = self._read_accel_gyro_from_imu()
        else:
            timestamp, gyro = self._read_gyro_from_imu()
            accel = None

        self.last_read_time = timestamp  # Last time stamp when data was read from the IMU

        return timestamp, accel, gyro

    def _sleep_until_next_sample(self):
        ''' Sleep for the remaining time in the sampling period '''
        sleep_time = self.imu_sample_period - (time.time() - self.last_read_time)
        if sleep_time > 0:
            time.sleep(min(sleep_time, self.MAX_SLEEP_TIME))

    def _read_gyro_from_imu(self) -> tuple[float, NdarrayNone]:
        timestamp = time.time()
        gyro = np.array(self.sensor.gyro)  # Gyroscope in rad/s

        if gyro is None:
            logger.warning("IMU: Failed to get gyro values")
            gyro = None
        
        return timestamp, gyro
    
    def _read_accel_gyro_from_imu(self) -> tuple[float, NdarrayNone, NdarrayNone]:
        timestamp = time.time()
        accel = np.array(self.sensor.acceleration)  # Acceleration in m/s^2
        gyro = np.array(self.sensor.gyro)  # Gyroscope in rad/s

        if accel is None:
            logger.warning("IMU: Failed to get accelerometer values")
            accel = None
            
        if gyro is None:
            logger.warning("IMU: Failed to get gyro values")
            gyro = None
        
        return timestamp, accel, gyro
    
    def _estimate_gyro_calibration(self) -> bool:
        """ 
        Determine the gyroscope offset by averaging over multipe samples while
        the IMU is stationary.
        """
        # TODO: Use GyroCalibration class
        self.gyro_offsets = [0.0, 0.0, 0.0]
        self.gyro_calibrated = True
        logger.info("Gyro calibrated")
        return True

    def _apply_gyro_calibration(self, gyro: np.ndarray):
        if self.gyro_calibrated and gyro is not None:
            return gyro - self.gyro_offsets
        else:
            return None

    def _check_valid_data(self, gyro: np.ndarray):
        """ 
        Check for outlier, limits, etc.
        TODO 
        """
        return True  # True if no outliers

    def _evaluate_moving_status(self, gyro: np.ndarray):
        """ TODO """
        return False

    def update(self) -> bool:
        ''' 
        Reads in the quaternion from the IMU. Returns True if a new valid
        sample is available. 
        '''
        # Read in the new raw samples
        timestamp, accel, gyro = self._read_raw_data()
        if gyro is None:
            return False  # Failed to get sensor values

        # Check validity of timestamp
        if timestamp <= self._prev_data_timestamp:
            logger.info("IMU: Previous timestamp same or older than current timestamp. Ignoring IMU sample.")
            return False

        # Check calibration status. If not calibrated, start calibration
        # NOTE: Accelerometer data is not calibrated
        if not self.gyro_calibrated:
            if not self._estimate_gyro_calibration():
                return False
        # Apply calibration
        gyro = self._apply_gyro_calibration(gyro)

        # Check if the new data is an outlier or has hit the range limits
        if not self._check_valid_data(gyro):
            return False

        # Determine if moving
        self._moving = self._evaluate_moving_status(gyro)

        # Construct quaternion
        if prev_quat is None:
            # TODO: Additional checks when prev_quat is None. E.g. Need to plate solve to sync
            prev_quat = np.quaternion(1, 0, 0, 0)  # Assume identity orientation
        quat = angular_velocity_to_quaternion(gyro, timestamp, self._prev_quat, self._prev_data_timestamp)

        # Valid sample obtained: Update current imu_data:
        self._prev_data_timestamp = self.imu_data.timestamp
        self._prev_quat = self.imu_data.quat
        self.imu_data.set(timestamp=timestamp, accel=accel, gyro=gyro, quat=quat)

        return True # New sample available

    def __str__(self):
        return (
            f"IMU Information:\n"
            f"Gyro data: ", self.gyro, "\n"
            f"Last Sample Time: {self.last_read_time}\n"
            f"Gyro calibration Status: {self.gyro_calibrated}\n"
            f"Gyro offsets: {self.gyro_offsets}\n"
            #f"Quaternion History: {self.quat_history}\n"
            #f"Average Quaternion: {self.avg_quat}\n"
            f"Moving: {self.moving}\n"
            #f"Reading Difference: {self.__reading_diff}\n"
            #f"Flip Count: {self._flip_count}\n"
            f"IMU Sample Frequency: {self.imu_sample_frequency}\n"
            f"Moving Threshold: {self.__moving_threshold}\n"
        )


@dataclass
class ImuRawData:
    """
    Data class for the raw data from the IMU
    """
    timestamp: Union[float, None] = None  # [s]
    accel: np.ndarray = None  # Accelerometer data in m/s^2
    gyro: NdarrayNone = None  # Gyroscope data in rad/s
    quat: quaternion.quaternion = None  # Quaternion relative to arbitrary reference (w, x, y, z)

    def set(timestamp: float, accel: NdarrayNone, gyro: NdarrayNone, quat: QuaternionNone):
        self.timestamp = timestamp
        self.accel = accel
        self.gyro = gyro
        self.quat = quat


def angular_velocity_to_quaternion(
        ang_vel: Union[np.ndarray, list],  # Angular velocity [rad/s]
        timestamp: float,  # Timestamp [s] of gyro measurement
        prev_quat: QuaternionNone,  # Previous quaternion
        prev_timestamp: float,  # Timestamp [s] of prev_quat
        ) -> QuaternionNone:
    """
    Integrate angular velocity measurements from the gyro into orientation
    quaternions relative to the previous quaternion.
    """
    MIN_OMEGA = 1E-12  # [rad/s] Minimum angular velocity (numerical noise)

    # Check inputs:
    if prev_quat is None:
        return None

    dt = timestamp - prev_timestamp
    if dt <= 0:
        logger.info("IMU: Previous timestamp same or older than current timestamp. Ignoring IMU sample.")
        return None

    omega = np.array(ang_vel)
    omega_norm = np.linalg.norm(omega)
    if omega_norm < MIN_OMEGA:
        return prev_quat.copy()  # No rotation (just numerical noise)

    # Calculate new quaternion
    theta = omega_norm * dt  # Rotation angle during timestep
    axis = omega / omega_norm  # Rotation axis
    half_theta = 0.5 * theta
    dq = quaternion.quaternion(np.cos(half_theta), *(axis * np.sin(half_theta)))
    quat = (q * dq).normalized()  # Apply incremental rotation

    return quat

class GyroCalibration:
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
        from PiFinder.imu.imu_fake import Imu as ImuFake

        imu = ImuFake()

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
