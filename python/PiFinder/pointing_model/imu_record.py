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
import time
#import numpy as np

class ImuSimple:
    def __init__(self):
        i2c = board.I2C()
        self.sensor = adafruit_bno055.BNO055_I2C(i2c)
        self.sensor.mode = adafruit_bno055.IMUPLUS_MODE
        # self.sensor.mode = adafruit_bno055.NDOF_MODE

        self.calibration = 0
  
        self.last_sample_time = time.time()

        # Calibration settings
        self.imu_sample_frequency = 1 / 30

    def update(self):
        ''' 
        Reads in the quaternion from the IMU. Returns True if a new valid
        sample is available. 
        '''
        new_sample_available = False

        # check for update frequency
        timestamp = time.time()
        if timestamp - self.last_sample_time < self.imu_sample_frequency:
            return new_sample_available  # Wait for full sampling period

        # adafruit_bno055 returns quaternion convention (w, x, y, z)
        quat = self.sensor.quaternion  # Returns tuple so won't be over-written
        self.last_sample_time = timestamp

        # Check IMU calibration status
        self.calibration = self.sensor.calibration_status[1]
        if self.calibration == 0:
            # logger.warning("NOIMU CAL")
            return new_sample_available  # IMU not calibrated

        if quat[0] is None:
            # logger.warning("IMU: Failed to get sensor values")
            return new_sample_available  # Failed to get sensor values

        # Valid sample obtained
        new_sample_available = True
        self.quat = quat  # Scalar-first quaternion: (w, x, y, z)
        self.timestamp = timestamp

        return new_sample_available


class RecordDataStream:
    '''
    Records data from multiple sensors and writes to a file using jsonl.
    '''
    BUFFER_SIZE = 1000  # Flush the buffer every BUFFER_SIZE samples
    # Data types:
    DATA_TYPE_DATETIME = 0
    DATA_TYPE_IMU_QUAT = 1
    DATA_TYPE_PLATE_SOLVE = 2

    def __init__(self, file_name="pifinder_recording.jsonl"):
        ''' Note that this will append to an existing file '''
        self.file_name = file_name
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


"""
# Commented out because difficulty installing pyarrow with pip on the Raspberry Pi 

import pyarrow as pa
import pyarrow.parquet as pq

class RecordDataStreamParquet:
    '''
    Records data from multiple sensors and writes to a Parquet file.
    '''
    BUFFER_SIZE = 100  # Flush the buffer every BUFFER_SIZE samples
    # Data types:
    DATA_TYPE_IMU_QUAT = 1
    DATA_TYPE_PLATE_SOLVE = 2

    # Define a unified schema with a struct for all possible sensor fields
    schema = pa.schema([
        ("data_type", pa.int64()),
        ("timestamp", pa.timestamp("s")),
        ("data", pa.struct([
            ("qw", pa.float64()),    # Quaternion qw 
            ("qx", pa.float64()),       
            ("qy", pa.float64()),        
            ("qz", pa.float64()),       
            ("RA", pa.float64()),  # RA
            ("Dec", pa.float64()),  # Dec
            ("Roll", pa.float64()),  # Roll
        ]))
    ])

    def __init__(self, file_name="pifinder_recording.parquet"):
        self.file_name = file_name
        self.buffer = []

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

    def flush_buffer(self):
        ''' Write the buffer to a Parquet file and clear the buffer '''
        if not self.buffer:
            return
        
        table = pa.Table.from_pylist(self.buffer, schema=self.schema)
        pq.write_table(table, self.file_name, append=True)  # Append to file
        self.buffer = []  # Flush the buffer
"""


def imu_monitor():
    imu = ImuSimple()
    record = RecordDataStream(file_name="imu_recording.jsonl")

    n_samples = 0
    while True:
        if imu.update():
            #print(
            #    f"IMU: quat={imu.quat}, time={imu.timestamp:.3f}"
            #)
            record.store_imu_quaternion(imu.timestamp, imu.quat)
            n_samples += 1
            if n_samples % 100 == 0:
                print(f"Recorded {n_samples} IMU samples...")
            if n_samples >= 500:
                record.stop_recording()
                break


if __name__ == "__main__":
    imu_monitor()
