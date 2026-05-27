import quaternion  # Numpy quaternion
import numpy as np
from typing import Union

# Type hint aliases
NdarrayNone = Union[np.ndarray, None]
QuaternionNone = Union[quaternion.quaternion, None]


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

    def __str__(self):
        return (f"ImuData(timestamp={self.timestamp}, accel={self.accel}, "
                f"gyro={self.gyro}, quat={self.quat})")
