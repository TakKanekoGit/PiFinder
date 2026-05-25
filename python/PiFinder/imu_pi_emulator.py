"""
IMU Sensor Emulator
"""

import numpy as np


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

