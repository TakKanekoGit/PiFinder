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

from PiFinder.multiproclogging import MultiprocLogging
from PiFinder.imu.imu_data import ImuData
from PiFinder.imu.imu import Imu

logger = logging.getLogger("IMU.pi")

# Type hint aliases
NdarrayNone = Union[np.ndarray, None]
QuaternionNone = Union[quaternion.quaternion, None]

QUEUE_LEN = 10  # TODO: Remove?


def imu_monitor(shared_state, console_queue, log_queue, enable_accel=False):
    MultiprocLogging.configurer(log_queue)
    logger.debug("Starting IMU")
    imu = None
    try:
        imu = Imu(enable_accel=enable_accel, emulate=False)
    except Exception as e:
        logger.error(f"Error starting phyiscal IMU : {e}")
        logger.error("Falling back to fake IMU")
        console_queue.put("IMU: Error starting physical IMU, using fake IMU")
        console_queue.put("DEGRADED_OPS IMU")
        # Run Imu() in emulation mode
        imu = Imu(enable_accel=enable_accel, emulate=True)

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

    #ii = 0  # FOR DEBUGGING
    while True:
        # Read IMU data, check validity, apply calibration and process 
        imu.update()
        
        if imu.gyro_calibration.valid:
            # Copy data to shared state. TODO: Use a standard class?
            imu_data["moving"] = imu.moving_status.moving
            imu_data["quat"] = imu.imu_data.quat
            imu_data["status"] = imu.gyro_calibration.valid
            # TODO: move_start and move_end don't seem to be used?
            imu_data["move_start"] = None
            imu_data["move_end"] = None

            if shared_state is not None:
                shared_state.set_imu(imu_data)

            # FOR DEBUGGING:
            #if ii % 10 == 0:
            #    print(f"IMU: Gyro: {np.rad2deg(imu.imu_data.gyro)} deg/s   Quat: {imu.imu_data.quat}")
            #    ii = 0    
            #ii += 1

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
