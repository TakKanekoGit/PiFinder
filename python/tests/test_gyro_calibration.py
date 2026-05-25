# python/PiFinder/imu/test_gyro_calibration.py

import numpy as np
import pytest

from PiFinder.imu.gyro_calibration import GyroCalibration

def test_gyro_calibration_estimate():
    # Test the estimate method of the GyroCalibration class
    gyro_calib = GyroCalibration()
    gyro_calib.estimate()
    assert gyro_calib.offsets is not None
    assert isinstance(gyro_calib.offsets, np.ndarray)
    assert gyro_calib.valid is True

def test_gyro_calibration_apply():
    # Test the apply method of the GyroCalibration class
    gyro_calib = GyroCalibration()
    gyro_calib.estimate()
    gyro = np.array([1.0, 2.0, 3.0])
    result = gyro_calib.apply(gyro)
    assert result is not None
    assert np.array_equal(result, gyro - gyro_calib.offsets)
