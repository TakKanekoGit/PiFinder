"""
Test imu_pi.py
"""

import numpy as np
import pytest
import quaternion
import time

from PiFinder.imu.imu_pi import Imu, ImuData, ImuMovingStatus, GyroCalibration

# Test Imu class:

@pytest.fixture(params=[True, False])
def setup_imu(request):
    # This method will run once before all tests
    return Imu(enable_accel=request.param, emulate=True)

def test_init(setup_imu):
    # Test the initialization of the Imu class
    assert isinstance(setup_imu, Imu)
    assert setup_imu.sensor is not None

def test_read_accel_gyro_from_imu(setup_imu):
    # Test the _read_accel_gyro_from_imu method
    if setup_imu.accel_enabled:
        timestamp, accel, gyro = setup_imu._read_accel_gyro_from_imu()
        assert isinstance(timestamp, float)
        assert isinstance(gyro, np.ndarray) and gyro is not None
        assert isinstance(accel, np.ndarray) and accel is not None

def test_read_gyro_from_imu(setup_imu):
    # Test the _read_gyro_from_imu method
    timestamp, gyro = setup_imu._read_gyro_from_imu()
    assert isinstance(timestamp, float)
    assert isinstance(gyro, np.ndarray) and gyro is not None

def test_read_raw_data(setup_imu):
    # Test the _read_raw_data method
    timestamp, accel, gyro = setup_imu._read_raw_data()
    assert isinstance(timestamp, float)
    assert isinstance(gyro, np.ndarray) and gyro is not None
    if setup_imu.accel_enabled:
        assert isinstance(accel, np.ndarray) and accel is not None

def test_update(setup_imu):
    # Test the update method
    assert setup_imu.update() is True
    
# Test other classes
"""
def test_ImuData():
    # Test the ImuData class
    imu_data = ImuData()
    assert isinstance(imu_data, ImuData)
    imu_data = ImuData(timestamp=time.time(), accel=None, gyro=None)

def test_ImuMovingStatus():
    imu_moving_status = ImuMovingStatus()
    imu_moving_status.update_thresholds()
    #imu_threshold_scale = cfg.get_option("imu_threshold_scale", 1)
    #imu_moving_status.update(self, imu_data: ImuData)
"""