# python/PiFinder/imu/test_gyro_calibration.py

import numpy as np
import pytest

from PiFinder.imu.gyro_calibration import GyroCalibration


@pytest.fixture
def gyro_calib():
    # This method will run once before all tests
    return GyroCalibration()

def test_reset(gyro_calib):
    gyro_calib.reset()

def test_set_offsets(gyro_calib):
    # Valid calibration offsets:
    gyro_calib.set_offsets([1.0, 2.0, 3.0])
    assert gyro_calib.offsets is not None
    assert gyro_calib.valid

    # None offsets:
    gyro_calib.set_offsets(NotImplemented)
    assert gyro_calib.offsets is None
    assert not gyro_calib.valid 

    # Invalid offsets:
    gyro_calib.set_offsets([1.0, 2.0])
    assert gyro_calib.offsets is None
    assert not gyro_calib.valid 


def test_gyro_calibration_estimate(gyro_calib):
    # Test the estimate method of the GyroCalibration class
    gyro_calib.estimate()
    assert gyro_calib.offsets is not None
    assert isinstance(gyro_calib.offsets, np.ndarray)
    assert gyro_calib.valid is True

def test_gyro_calibration_apply(gyro_calib):
    # Test the apply method of the GyroCalibration class
    gyro_calib.estimate()
    gyro = np.array([1.0, 2.0, 3.0])
    result = gyro_calib.apply(gyro)
    assert result is not None
    assert np.array_equal(result, gyro - gyro_calib.offsets)
