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

logger = logging.getLogger("IMU.pi")

# Type hint aliases
NdarrayNone = Union[np.ndarray, None]
QuaternionNone = Union[quaternion.quaternion, None]

QUEUE_LEN = 10  # TODO: Remove?


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
        # TODO: Replace with Imu() in emulation mode
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
        # Read IMU data, check validity, apply calibration and process 
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
