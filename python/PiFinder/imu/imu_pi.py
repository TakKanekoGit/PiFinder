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
from PiFinder.imu.gyro_calibration import GyroCalibration

logger = logging.getLogger("IMU.pi")

# Type hint aliases
NdarrayNone = Union[np.ndarray, None]
QuaternionNone = Union[quaternion.quaternion, None]

QUEUE_LEN = 10  # TODO: Remove?


class ImuData:
    """
    Data class for the raw data from the IMU
    """
    # Raw data
    timestamp: Union[float, None] = None  # [s]
    accel: NdarrayNone = None  # Accelerometer data in m/s^2
    gyro: NdarrayNone = None  # Gyroscope data in rad/s
    quat: quaternion.quaternion = None  # Quaternion relative to arbitrary reference (w, x, y, z)

    def __init__(self, timestamp: float, 
                 accel: NdarrayNone = None, 
                 gyro: NdarrayNone = None, 
                 quat: QuaternionNone = None):
        self.timestamp = timestamp
        self.accel = accel
        self.gyro = gyro
        self.quat = quat

    def set(self, timestamp: float, accel: NdarrayNone = None, 
            gyro: NdarrayNone = None, quat: QuaternionNone = None):
        self.timestamp = timestamp
        self.accel = accel
        self.gyro = gyro
        self.quat = quat

    @property
    def ang_vel(self) -> Union[float, None]:
        """ Angular velocity from gyro [rad/s] """
        if self.gyro is None:
            return None
        else:
            return np.linalg.norm(self.gyro)


class ImuMovingStatus:
    """
    Whether the IMU is moving above some threshold
    """
    # Default lower & upper hysteresis thresholds for movement detection [rad/s] 
    DEFAULT_MOVING_ANG_VEL_THRESHOLDS = [np.deg2rad(1.0), np.deg2rad(1.8)]

    moving: bool  # Whether the IMU is moving
    timestamp: Union[float, None]  # [s] Time when the IMU started moving
    stationary_since: float  # [s] Time when the IMU stopped moving

    _moving_ang_vel_thresholds: list[2]  # [lower, upper] ang. vel. thresholds [rad/s]

    def __init__(self):
        self.moving = False
        self.timestamp = None
        self.stationary_since = time.time()
        
        # Init. upper & lower hysteresis thresholds for movement detection [rad/s] 
        # TODO: Does need need to updated if cfg gets updated? 
        self.update_thresholds()

    def update_thresholds(self):
        """ 
        Updates the moving thresholds (in rad/s) scaled by the user configuration
        imu_threshold_scale. Returns a list of [lower, upper] thresholds for
        hyesteresis.
        """
        assert self.DEFAULT_MOVING_ANG_VEL_THRESHOLDS[0]  < self.DEFAULT_MOVING_ANG_VEL_THRESHOLDS[1]

        cfg = config.Config()
        imu_threshold_scale = cfg.get_option("imu_threshold_scale", 1)
        assert isinstance(imu_threshold_scale, (float, int))
        assert imu_threshold_scale > 0

        self._moving_ang_vel_thresholds = [thr * imu_threshold_scale for thr in self.DEFAULT_MOVING_ANG_VEL_THRESHOLDS]

    def update(self, imu_data: ImuData):
        """ 
        Determine and update the moving status based on the gyro angular
        velocity. The threshold has hysteresis for robustness and to prevent
        oscillations. If gyro data is invalid, leaves the state as is.
        """
        self.timestamp = imu_data.timestamp
        if imu_data.gyro is not None:
            if self.moving:
                if imu_data.ang_vel < self._moving_ang_vel_thresholds[0]:
                    self.moving = False
                    self.stationary_since = imu_data.timestamp
            else:
                if imu_data.ang_vel > self._moving_ang_vel_thresholds[1]:
                    self.moving = True


class Imu:
    """
    Previous version modified the IMU axes but the IMU now outputs the
    measurements using its native axes and the transformation from the IMU
    axes to the camera frame is done by the IMU dead-reckonig functionality.
    
    NOTE: Not all accelerometer features available but included for development
    """
    IMU_SAMPLE_FREQUENCY = 1 / 30  # Should be > 2 * sensor bandwidth
    MAX_SLEEP_TIME = 1.0  # [s] Max sleep time to avoid sleeping for too long
    N_GYRO_CAL_SAMPLES = 150

    accel_enabled: bool
    imu_sample_period: float
    imu_data: ImuData
    gyro_calibration: GyroCalibration

    # Internal states:
    moving_state: ImuMovingStatus
    _last_read_time: float  # Last time stamp when data was read from the IMU (but not necessarily valid and stored)
    _prev_quat: QuaternionNone  # Previous quaternion state
    _prev_data_timestamp: float   # Timestamp associated with _prev_quat

    def __init__(self, enable_accel: bool = False, emulate: bool = False):
        self.sensor = self._init_sensor(enable_accel=enable_accel, emulate=emulate)
        
        self.accel_enabled = enable_accel
        self.imu_sample_period = 1 / self.IMU_SAMPLE_FREQUENCY

        self.imu_data = ImuData(timestamp=None)
        self.gyro_calibration = GyroCalibration()

        # Internal states
        self.moving_state = ImuMovingStatus()
        self._last_read_time = time.time()
        self._prev_quat = None
        self._prev_data_timestamp = time.time()

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
        if time.time() - self._last_read_time < self.imu_sample_period:
            self._sleep_until_next_sample()

        # Read in the new sample
        if self.accel_enabled:
            timestamp, accel, gyro = self._read_accel_gyro_from_imu()
        else:
            timestamp, gyro = self._read_gyro_from_imu()
            accel = None

        self._last_read_time = timestamp  # Last time stamp when data was read from the IMU

        return timestamp, accel, gyro

    def _sleep_until_next_sample(self):
        ''' Sleep for the remaining time in the sampling period '''
        sleep_time = self.imu_sample_period - (time.time() - self._last_read_time)
        if sleep_time > 0:
            time.sleep(min(sleep_time, self.MAX_SLEEP_TIME))

    def _read_gyro_from_imu(self) -> tuple[float, NdarrayNone]:
        timestamp = time.time()
        gyro = self.sensor.gyro  # Gyroscope in rad/s
        
        if gyro is None:
            logger.warning("IMU: Failed to get gyro values")
            gyro = None
        elif isinstance(gyro, list):
            gyro = np.array(gyro)
        else:
            raise ValueError("Unexpected gyro type: " + type(gyro))
        
        return timestamp, gyro
    
    def _read_accel_gyro_from_imu(self) -> tuple[float, NdarrayNone, NdarrayNone]:
        accel = self.sensor.acceleration  # Acceleration in m/s^2
        timestamp, gyro = self._read_gyro_from_imu()  # Gyroscope in rad/s

        if accel is None:
            logger.warning("IMU: Failed to get accelerometer values")
            accel = None
        elif isinstance(accel, list):
            accel = np.array(accel)
        else:
            raise ValueError("Unexpected accel type: " + type(accel))   
        
        return timestamp, accel, gyro

    def _check_valid_data(self, gyro: np.ndarray):
        """ 
        Check for outlier, limits, etc.
        TODO 
        """
        return True  # True if no outliers

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
        # TODO: Deal with the case if this happens repeatedly because time has jumped
        if timestamp <= self._prev_data_timestamp:
            logger.info("IMU: Previous timestamp same or older than current timestamp. Ignoring IMU sample.")
            return False

        # Check calibration status. If not calibrated, start calibration
        # NOTE: Accelerometer data is not calibrated
        if not self.gyro_calibration.valid:
            # Start calibration
            if self.gyro_calibration.estimate():
                logger.info(f"Gyro calibrated. Offsets = {self.gyro_calibration.offsets}")
            else:
                logger.info("Gyro calibration failed")
                return False  # Calibration failed
        # Apply calibration
        gyro = self.gyro_calibration.apply(gyro)

        # Check if the new data is an outlier or has hit the range limits
        if not self._check_valid_data(gyro):
            return False

        # Construct quaternion
        if self._prev_quat is None:
            # TODO: Additional checks when _prev_quat is None. E.g. Need to plate solve to sync
            self._prev_quat = np.quaternion(1, 0, 0, 0)  # Assume identity orientation
        quat = angular_velocity_to_quaternion(gyro, timestamp, self._prev_quat, self._prev_data_timestamp)

        assert isinstance(timestamp, float)
        self.imu_data.set(timestamp, accel=accel, gyro=gyro, quat=quat)
        self.moving_state.update(self.imu_data)  # Determine if moving

        # Valid sample obtained: Update previous values
        self._prev_data_timestamp = self.imu_data.timestamp
        self._prev_quat = self.imu_data.quat

        return True # New sample available

    def __str__(self):
        return (
            f"IMU Information:\n"
            f"Gyro data: ", self.gyro, "\n"
            f"Last Sample Time: {self._last_read_time}\n"
            f"Gyro calibration Status: {self.gyro_calibrated}\n"
            #f"Gyro offsets: {self.gyro_offsets}\n"
            #f"Quaternion History: {self.quat_history}\n"
            #f"Average Quaternion: {self.avg_quat}\n"
            f"Moving: {self.moving_status.moving}\n"
            #f"Reading Difference: {self.__reading_diff}\n"
            #f"Flip Count: {self._flip_count}\n"
            f"IMU Sample Frequency: {self.IMU_SAMPLE_FREQUENCY}\n"
            #f"Moving Threshold: {self.__moving_threshold}\n"
        )


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


def imu_monitor(shared_state, console_queue, log_queue):
    MultiprocLogging.configurer(log_queue)
    logger.debug("Starting IMU")
    imu = None
    try:
        imu = Imu(enable_accel=True)
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
