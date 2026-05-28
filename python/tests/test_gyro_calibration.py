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
    # Test calibration estimate: Constant input
    assert gyro_calib.valid is False

    gyro = np.deg2rad([0.1, 0.2, 0.3])
    for ii in range(GyroCalibration.N_GYRO_CAL_SAMPLES):  
        gyro_calib.estimate(gyro)  # Fill buffer with N_GYRO_CAL_SAMPLES
    assert gyro_calib._buffer_full

    breakpoint()
    assert gyro_calib.valid
    assert gyro_calib.offsets is not None
    assert isinstance(gyro_calib.offsets, np.ndarray)
    assert np.all(gyro_calib.offsets == gyro)

def test_gyro_calibration_apply(gyro_calib):
    # Test the apply method of the GyroCalibration class
    gyro_calib.set_offsets([0.1, 0.2, 0.3])
    assert gyro_calib.valid

    gyro = np.deg2rad([0.1, 0.2, 0.3])
    gyro_calib.estimate(gyro)
    result = gyro_calib.apply(gyro)
    assert result is not None
    assert np.array_equal(result, gyro - gyro_calib.offsets)
