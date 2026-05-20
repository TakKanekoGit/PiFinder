#!/usr/bin/python
# -*- coding:utf-8 -*-
'''
For testing the IMU: Not for use by PiFinder main loop.
Prints the IMU measurements (based on imu_pi.py)

TODO: Remove this in the future.
'''

import adafruit_bno055
import board
import datetime
import json
from pathlib import Path
import time
#import numpy as np
import logging

logger = logging.getLogger(__name__)

class ImuSimple:
    def __init__(self):
        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.sensor.mode = adafruit_bno055.ACCGYRO_MODE

        self.gyro = None  # Gyroscope data in rad/s
        self.accel = None  # Accelerometer data in m/s^2
        self.quat = None  # Quaternion data quaternion.quaternion(w, x, y, z)

        #self.gyro_calibration_status = 0
        #self.acc_calibration_status = 0
  
        self.last_sample_time = time.time()  # Last time stamp when data was read from the IMU (but not necessarily valid and stored)

        # Calibration settings
        self.imu_sample_frequency = 30.0
        self.imu_sample_period = 1 / self.imu_sample_frequency

    def update(self) -> bool:
        ''' 
        Reads in the quaternion from the IMU. Returns True if a new valid
        sample is available. 
        '''
        # check for a new sample period
        timestamp = time.time()
        if timestamp - self.last_sample_time < self.imu_sample_period:
            return False  # Not the end of the full sampling period
        self.last_sample_time = timestamp

        accel = self.sensor.acceleration  # Acceleration in m/s^2
        gyro = self.sensor.gyro  # Gyroscope in rad/s

        """
        # Check BNO055's internal calibration status
        _, self.gyro_calibration_status, self.acc_calibration_status , _ \
            = self.sensor.calibration_status  # (sys, gyro, accel, mag)
        print(self.sensor.calibration_status)
        if self.gyro_calibration_status == 0:
            logger.warning("IMU: Gyro not calibrated")
            return False  # Gyro not calibrated
        # TODO: Check accelerometer cal status when it's used in future
        """

        if gyro[0] is None:
            # logger.warning("IMU: Failed to get sensor values")
            return False  # Failed to get sensor values

        # Valid sample obtained
        self.gyro = gyro  # Gyroscope data in rad/s
        self.accel = accel  # Accelerometer data in m/s^2
        self.timestamp = timestamp

        return True # New sample available

    def sleep(self):
        ''' Sleep for the remaining time in the sampling period '''
        sleep_time = self.imu_sample_period - (time.time() - self.last_sample_time)
        if sleep_time > 0:
            time.sleep(sleep_time)


class RecordDataStream:
    '''
    Records data from multiple sensors and writes to a file using jsonl.
    '''
    BUFFER_SIZE = 1000  # Flush the buffer every BUFFER_SIZE samples
    # Data types:
    DATA_TYPE_DATETIME = 0
    DATA_TYPE_IMU_QUAT = 1
    DATA_TYPE_PLATE_SOLVE = 2

    def __init__(self, path=Path("pifinder_recording.jsonl")):
        ''' Note that this will append to an existing file '''
        print(f"Recording to: {path}")
        self.file_name = path
        self.buffer = []
        self.n_records = 0  # Counter to the number of records written
        self.store_current_datetime()  # Store the start time

    def store_current_datetime(self):
        ''' Store the current UTC time to the buffer '''
        now = datetime.datetime.now(datetime.timezone.utc)
        self.buffer.append({
            "data_type": self.DATA_TYPE_DATETIME, 
            "timestamp": time.time(), 
            "data": {
                "date": now.strftime("%Y%m%d"), 
                "time": now.strftime("%H:%M:%S"),
                "timezone": "UTC",
                }
            })
        self.check_buffer_size_and_flush()

    def store_imu_quaternion(self, timestamp: float, quat: tuple):
        ''' Store IMU quaternion data to the buffer '''
        self.buffer.append({
            "data_type": self.DATA_TYPE_IMU_QUAT, 
            "timestamp": timestamp, 
            "data": {"qw": quat[0], "qx": quat[1], "qy": quat[2], "qz": quat[3]}
            })
        self.check_buffer_size_and_flush()

    def store_plate_solving(self, timestamp: float, RA: float, Dec: float, Roll: float):
        ''' Store IMU quaternion data to the buffer '''
        self.buffer.append({
            "data_type": self.DATA_TYPE_IMU_QUAT, 
            "timestamp": timestamp, 
            "data": {"RA": RA, "Dec": Dec, "Roll": Roll}
            })
        self.check_buffer_size_and_flush()

    def check_buffer_size_and_flush(self):
        ''' Check if the buffer size exceeds the limit and flush if needed '''
        if len(self.buffer) >= self.BUFFER_SIZE:
            self.flush_buffer()

    def stop_recording(self):
        ''' 
        Call this to stop recording and flush the buffer. Make sure to call
        this before exiting the program to avoid losing the last set of data in
        the buffer. 
        '''
        self.store_current_datetime()  # Store the end time
        self.flush_buffer() 

    def flush_buffer(self):
        ''' 
        Write the buffer to a Parquet file and clear the buffer. 
        '''
        if not self.buffer:
            return
        
        with open(self.file_name, "a") as f:
            self.n_records += len(self.buffer)
            for record in self.buffer:
                f.write(json.dumps(record) + "\n")
        
        self.buffer = []  # Flush the buffer


def imu_monitor(save_data=True):

    if save_data:
        dir = Path("~/PiFinder_data/telemetry")
        if not Path(dir).exists():
            print(f"Directory {dir} does not exist. Creating it.")
            Path(dir).mkdir(parents=True, exist_ok=True)
        time_str = time.strftime("%Y%m%d%H%M")

        record = RecordDataStream(path=dir / f"{time_str}-imu_recording.jsonl")
    else:
        record = None

    imu = ImuSimple()
    max_samples = 100
    update_interval = 500

    n_samples = 0
    while True:
        if imu.update():
            if save_data:
                record.store_imu_quaternion(imu.timestamp, imu.quat)
            else:
                print(
                    f"IMU: gyro={imu.gyro}, acc={imu.accel}, time={imu.timestamp:.3f}"
                )

            n_samples += 1
            if n_samples % update_interval == 0:
                print(f"Recorded {n_samples} IMU samples...")
            if n_samples >= max_samples:
                if save_data:
                    record.stop_recording()
                break
        else:
            imu.sleep()  # Sleep for the remaining time in the sampling period


if __name__ == "__main__":
    imu_monitor(save_data=False)
