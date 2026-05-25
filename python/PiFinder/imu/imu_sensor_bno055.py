""" 
Configures the BNO055 IMU sensor 
"""
import board
import adafruit_bno055
import logging

logger = logging.getLogger("imu_bno055")


def configure_imu_sensor_bno055(enable_accel: bool):
    logger.info("IMU: Starting BNO055 IMU")
    i2c = board.I2C()
    sensor = adafruit_bno055.BNO055_I2C(i2c)

    if enable_accel:
        sensor = _configure_imu_accgyro_mode(sensor)
    else:
        sensor = _configure_imu_gyro_mode(sensor)
    return sensor


def _configure_imu_accgyro_mode(
    sensor: adafruit_bno055.BNO055_I2C) -> adafruit_bno055.BNO055_I2C:
    """
    Configure the IMU.
    """
    # ACCGYRO mode: Accelerometer + Gyro data only, no fusion
    sensor.mode = adafruit_bno055.ACCGYRO_MODE
    sensor = _configure_gyro(sensor)
    sensor = _configure_accelerometer(sensor)

    logger.info("IMU: Configured in ACCGYRO mode")
    return sensor


def _configure_imu_gyro_mode(
    sensor: adafruit_bno055.BNO055_I2C) -> adafruit_bno055.BNO055_I2C:
    """
    Configure the IMU.
    """
    # ACCGYRO mode: Accelerometer + Gyro data only, no fusion
    sensor.mode = adafruit_bno055.GYRONLY_MODE
    sensor = _configure_gyro(sensor)

    logger.info("IMU: Configured in GYRONLY mode")
    return sensor


def _configure_gyro(sensor: adafruit_bno055.BNO055_I2C) -> adafruit_bno055.BNO055_I2C:
    """
    Configure the gyro sensitivity (range) and bandwidth.

    Gyro range options:
    GYRO_2000_DPS, GYRO_1000_DPS, GYRO_500_DPS, GYRO_250_DPS, GYRO_125_DPS

    Gyro bandwidth options:
    GYRO_523HZ, GYRO_230HZ, GYRO_116HZ, GYRO_47HZ, GYRO_23HZ, GYRO_12HZ
    GYRO_64HZ, GYRO_32HZ
    """
    # Configure accelerometer: Range +/-4G, Bandwidth 7.81Hz
    sensor.accel_range = adafruit_bno055.ACCEL_4G
    sensor.accel_bandwidth = adafruit_bno055.ACCEL_7_81HZ
    logger.info("IMU: Cofigured accelerometer range and bandwidth")
    return sensor


def _configure_accelerometer(sensor: adafruit_bno055.BNO055_I2C) -> adafruit_bno055.BNO055_I2C:
    """
    Configure the accelerometer sensitivity (range) and bandwidth.

    Accelerometer range options:
    ACCEL_2G, ACCEL_4G, ACCEL_8G, ACCEL_16G

    Bandwidth options:
    ACCEL_7_81HZ, ACCEL_15_63HZ, ACCEL_31_25HZ, ACCEL_62_5HZ,
    ACCEL_125HZ, ACCEL_250HZ, ACCEL_500HZ, ACCEL_1000HZ
    """
    # Configure gyro: Range +/-500 dps, Bandwidth 12Hz
    sensor.gyro_range = adafruit_bno055.GYRO_500_DPS
    sensor.gyro_bandwidth = adafruit_bno055.GYRO_12HZ
    logger.info("IMU: Cofigured gyro range and bandwidth")
    return sensor
