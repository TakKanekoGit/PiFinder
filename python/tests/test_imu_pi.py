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

def test_str(setup_imu):
    # Test the __str__ method
    assert isinstance(setup_imu.__str__(), str)

# Test other classes

def test_ImuData():
    # Test: ImuData
    imu_data = ImuData(time.time())
    assert isinstance(imu_data, ImuData)
    imu_data = ImuData(time.time(), accel=None, gyro=np.array([1, 2, 3]))
    assert isinstance(imu_data, ImuData)
    assert imu_data.gyro is not None
    assert imu_data.ang_vel == np.linalg.norm(imu_data.gyro)
    assert isinstance(imu_data.__str__(), str)


def test_ImuMovingStatus():
    # Test: ImuMovingStatus
    imu_moving_status = ImuMovingStatus()
    imu_moving_status.update_thresholds()

    imu_data = ImuData(time.time(), accel=None, gyro=np.array([0, 0, 0]))
    imu_moving_status.update(imu_data)
    assert imu_moving_status.moving is False

    rad_vel = imu_moving_status._moving_ang_vel_thresholds[1]
    imu_data = ImuData(time.time(), accel=None, gyro=np.array([rad_vel, rad_vel, rad_vel]))
    imu_moving_status.update(imu_data)
    assert imu_moving_status.moving is True
