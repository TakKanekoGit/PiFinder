#!/usr/bin/python
# -*- coding:utf-8 -*-
"""
For testing the IMU: Not for use by PiFinder main loop.
Prints the IMU measurements (based on imu_pi.py)

TODO: Remove this in the future.
"""

import time
import board
import adafruit_bno055
import numpy as np
# import logging


# from PiFinder import config

# logger = logging.getLogger("IMU.pi")

QUEUE_LEN = 10
MOVE_CHECK_LEN = 2


class ImuSimple:
    def __init__(self):
        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.sensor.mode = adafruit_bno055.IMUPLUS_MODE
        # self.sensor.mode = adafruit_bno055.NDOF_MODE

        self.calibration = 0
  
        self.last_sample_time = time.time()

        # Calibration settings
        self.imu_sample_frequency = 1 / 30

    def update(self):
        """ 
        Reads in the quaternion from the IMU. Returns True if a new valid
        sample is available. 
        """
        new_sample_available = False

        # check for update frequency
        timestamp = time.time()
        if timestamp - self.last_sample_time < self.imu_sample_frequency:
            return new_sample_available  # Wait for full sampling period

        # adafruit_bno055 returns quaternion convention (w, x, y, z)
        quat = self.sensor.quaternion  # Returns tuple so won't be over-written
        self.last_sample_time = timestamp

        # Check IMU calibration status
        self.calibration = self.sensor.calibration_status[1]
        if self.calibration == 0:
            # logger.warning("NOIMU CAL")
            return new_sample_available  # IMU not calibrated

        if quat[0] is None:
            # logger.warning("IMU: Failed to get sensor values")
            return new_sample_available  # Failed to get sensor values

        # Valid sample obtained
        new_sample_available = True
        self.quat = quat  # Scalar-first quaternion: (w, x, y, z)
        self.timestamp = timestamp

        return new_sample_available


def imu_monitor():
    # MultiprocLogging.configurer(log_queue)
    imu = ImuSimple()

    while True:
        if imu.update():
            print(
                f"IMU: quat={imu.quat}, time={imu.timestamp:.3f}"
            )
        
if __name__ == "__main__":
    imu_monitor()
