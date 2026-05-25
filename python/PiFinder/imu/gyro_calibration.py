"""
Estimates the gyro offsets calibration and applies the calibration.
"""

import numpy as np
from typing import Union


class GyroCalibration:
    """ 
    Calibrate the gyro by storing N measurements while stationary and taking
    the averages.
    """
    offsets: Union[np.ndarray, None]
    valid: bool  # Has valid calibration

    def __init__(self):
        self.offsets = None
        self.valid = False

    def estimate(self) -> bool:
        """ 
        Determine the gyroscope offset by averaging over multipe samples while
        the IMU is stationary.
        """
        # TODO: Use GyroCalibration class
        self.offsets = np.array([0.0, 0.0, 0.0])
        self.valid = True
        return True

    def apply(self, gyro: np.ndarray):
        if self.valid and gyro is not None:
            return gyro - self.offsets
        else:
            return None

