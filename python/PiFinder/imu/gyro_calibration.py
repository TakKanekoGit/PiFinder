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
    N_GYRO_CAL_SAMPLES = 150

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
        if gyro is None:
            return None  # Invalid input
        elif not self.valid:
            return None  # Not calibrated
        else:
            assert isinstance(self.offsets, np.ndarray)
            assert isinstance(gyro, np.ndarray)
            return gyro - self.offsets
