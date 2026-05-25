"""
IMU Sensor Emulator
"""
import numpy as np
import time
import logging

logger = logging.getLogger("imu_bno055")

def configure_imu_sensor_emulator(enable_accel: bool):
    logger.info("IMU: Starting IMU in emulation mode")

    if enable_accel:
        sensor = ImuSensorEmulator()
    else:
        raise NotImplementedError("Gyro emulation not implemented")

    return sensor

class ImuSensorEmulator(self):
    """ 
    Emulates the IMU. Imu() can call this class by:
    
    imu = Imu()
    imu.sensor = ImuSensorEmulator()
    """

    def __init__(self):
        pass

    def acceleration(self) -> list[float]:
        return [0.0, 0.0, 0.0]

    def gyro(self) -> list[float]:
        return [0.0, 0.0, 0.0]

