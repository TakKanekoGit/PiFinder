"""
Configure the IMU Sensor Emulator
"""
import logging

logger = logging.getLogger("imu_bno055")

def configure_imu_sensor_emulator(enable_accel: bool):
    logger.info("IMU: Starting IMU in emulation mode")
    sensor = ImuSensorEmulator(emulate_acceleration=enable_accel, emulate_gyro=True)
    return sensor

class ImuSensorEmulator:
    """ 
    Emulates the IMU in place of:

    i2c = board.I2C()
    sensor = adafruit_bno055.BNO055_I2C(i2c)
    """
    emulate_acceleration: bool
    emulate_gyro: bool

    def __init__(self, emulate_acceleration: bool = True, emulate_gyro: bool = True):
        self.emulate_acceleration = emulate_acceleration
        self.emulate_gyro = emulate_gyro

    @property
    def acceleration(self) -> list[float]:
        """ Returns emulated acceleration values in m/s2 """
        assert self.emulate_acceleration, "Acceleration not emulated"
        return [0.0, 0.0, 9.8]  # [a_x, a_y, a_z]

    @property
    def gyro(self) -> list[float]:
        """ Returns emulated gyro values in rad/s """
        assert self.emulate_gyro, "Gyro not emulated"
        return [0.0, 0.0, 0.0]  # [w_x, w_y, w_z]

