"""
Estimates the gyro offsets calibration and applies the calibration.
"""
import logging
import numpy as np
from typing import Union
import time

from PiFinder.imu.imu_sensor_bno055 import GyroSpecs

NdarrayNone = Union[np.ndarray, None]

logger = logging.getLogger("GyroCalibration")


class GyroCalibration:
    """ 
    Calibrate the gyro by storing N measurements while stationary and taking
    the averages.
    """
    N_GYRO_CAL_SAMPLES = 150
    T_BETWEEN_CALIBRATION_ATTEMPTS = 1.0  # [s] Time to wait between calibration attempts

    offsets: Union[np.ndarray, None]
    valid: bool  # Has valid calibration
    _gyro_specs: GyroSpecs

    _buffer: np.ndarray  # Buffer to store the gyro samples to use for calibration
    _n_buffered: int  # Number of samples in the buffer
    _buffer_full: bool  # True when the buffer is full

    _noise_threshold: float  # Worst-case noise [rad/s]
    _stationary_threshold: float  # Threshold to deem as stationary for calibration [rad/s]

    _last_attempted_timestamp: float  # Last time the calibration was attempted

    def __init__(self):
        self.offsets = None
        self.valid = False
        self._gyro_specs = GyroSpecs()

        self._reset_buffer()

        # Expected worst-case offset & noise when gyro is stationary (with safety factors)
        self._noise_threshold = self._gyro_specs.output_noise * 5
        self._stationary_threshold = self._gyro_specs.zero_rate_offset * 1.5 + self._noise_threshold  # Ang. vel threshold [rad/s]

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

    def estimate(self, gyro: NdarrayNone) -> bool:
        """ 
        Determine the gyroscope offset by averaging over multipe samples while
        the IMU is stationary.
        """
        if self.valid:
            return True  # Already calibrated
        elif gyro is None:
            return False  # Invalid input
        
        # Still need to calibrate: Gather samples to calibrate
        assert len(gyro) == 3
        # Check if the gyro is stationary
        if np.all(np.abs(gyro) > self._stationary_threshold):
            # Moved. Reset the buffer & start again
            self._reset_buffer()
            return False
        
        # Below the worst case threshold (but could still be moving)
        self._append_to_buffer(gyro)
        if self._buffer_full == False:
            return False  # Get more samples

        # Buffer is full. Attempt to estimate the offsets
        dt = time.time() - self._last_attempted_timestamp
        if dt < 0:
            # dt shouldn't be negative. Time's been reset. Start again.
            self._reset_buffer()
            return False
            
        if dt < self.T_BETWEEN_CALIBRATION_ATTEMPTS:
            # Wait to fill up the buffer with fresh samples before attemping calibration
            return False
        else:
            if np.any(np.std(self._buffer, axis=0) > self._noise_threshold):
                # Likely moved if the SD exceeded the threshold. Continue gathering data.
                self._last_attempted_timestamp = time.time()
                logger.info("Gyro calibration failed. Likely moved.")
                return False
            else:
                avr = np.mean(self._buffer, axis=0)

                if np.all(np.abs(avr) <= self._stationary_threshold):
                    # Calibrated!
                    self.offsets = avr
                    self.valid = True
                    logger.info(f"Gyro calibrated. Offsets = {self.offsets}")
                    self._last_attempted_timestamp = time.time()
                    return True
                else:
                    # Measured offsets much higher than expected. Start again.
                    logger.warning("Gyro calibration failed. Offsets higher than expected.")
                    self._reset_buffer()
                    return False

        return False  # Shouldn't get to this point

    def apply(self, gyro: NdarrayNone) -> NdarrayNone:
        if gyro is None:
            return None  # Invalid input
        elif not self.valid:
            return None  # Not calibrated
        else:
            assert isinstance(self.offsets, np.ndarray)
            assert isinstance(gyro, np.ndarray)
            return gyro - self.offsets

    def _reset_buffer(self):
        self._buffer = np.zeros((self.N_GYRO_CAL_SAMPLES, 3))
        self._n_buffered = 0
        self._buffer_full = False
        self._last_attempted_timestamp = time.time()

    def _append_to_buffer(self, gyro):
        assert self._n_buffered <= self.N_GYRO_CAL_SAMPLES
        if self._buffer_full == False:
            # Append to buffer
            assert self._n_buffered <= self.N_GYRO_CAL_SAMPLES
            self._buffer[self._n_buffered, :] = gyro
            self._n_buffered += 1
            if self._n_buffered == self.N_GYRO_CAL_SAMPLES:
                self._buffer_full = True
        else:
            # Buffer is full. Replace oldest sample
            self._buffer[:-1, :] = self._buffer[1:, :]
            self._buffer[-1, :] = gyro
            self._buffer_full = True