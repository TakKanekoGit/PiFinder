"""
Estimates the gyro offsets calibration and applies the calibration.
"""
import logging
import numpy as np
from typing import Union

logger = logging.getLogger("GyroCalibration")


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

    def reset(self):
        self.offsets = None
        self.valid = False

    def set_offsets(self, offsets):
        if offsets is None:
            self.reset()
        else:
            try: 
                if len(offsets) == 3:
                    self.offsets = np.array(offsets)
                    self.valid = True
                else:
                    logger.error(f"Expected length 3 for gyro offsets. Got length {len(offsets)}")
                    self.reset()
            except Exception as e:
                logger.error(f"Expected length 3 for gyro offsets: {e}")
                self.reset()

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
