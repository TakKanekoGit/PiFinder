# python/tests/test_imu.py

import pytest
from unittest.mock import Mock, patch
from PiFinder.imu_pi import Imu

@pytest.mark.unit
class TestImu:
    @classmethod
    def setup_class(cls):
        # This method will run once before all tests
        cls.imu = Imu(emulate=True)

    def test_init(self):
        # Test the initialization of the Imu class
        assert self.imu.sensor is not None
        assert self.imu.accel_enabled is False

