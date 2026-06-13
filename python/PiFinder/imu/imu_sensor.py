from abc import ABC


class ImuSensor(ABC):
    """
    Base class to interface with different IMU sensors

    Enalbes interfacing to different IMUs and running in different modes in an
    uniform way. Some IMUs may not support all the methods.
    """
    sensor: object
    imu_mode: str

    gyro_enabled: bool = False
    gyro_bandwidth: float
    gyro_specs: GyroSpecs

    accelerometer_enabled: bool = False
    accelerometer_bandwidth: float

    def __init__(self):
        pass

    @abstractmethod
    def configure(self):
        pass

    def _reset_configurations(self):
        """ NOTE: This does not reset the sensor """
        self.sensor = None
        self.imu_mode = None

        self.gyro_enabled = False
        self.gyro_bandwidth = None
        self.gyro_specs = None

        self.accelerometer_enabled = False
        self.accelerometer_bandwidth = None 

    # ------ Configures the IMU mode ----------

    def _configure_accelerometer_mode(self):
        """
        Configure to accelerometer mode. Set the sensitivity (range) and
        data rate/bandwidth to the PiFinder default.
        """
        raise NotImplementedError("Method not implemented/available for this sensor")

    def _configure_gyro_mode(self):
        """
        Configure to gryo mode. Setthe sensitivity (range) and data
        rate/bandwidth to the PiFinder default.
        """
        raise NotImplementedError("Method not implemented/available for this sensor")

    def _configure_accgyro_mode(self):
        """
        Configure to accelerometer + gyro mode. Set the sensitivity (range)
        and data rate/bandwidth to the PiFinder default.
        """
        raise NotImplementedError("Method not implemented/available for this sensor")

    def _configure_fusion_mode(self):
        raise NotImplementedError("Method not implemented/available for this sensor")

    # -------- Set the range and data rate/bandwidth of each sensor ------

    def _set_gyro_range(self):
        raise NotImplementedError("Method not implemented/available for this sensor")

    def _set_gyro_data_rate(self):
        raise NotImplementedError("Method not implemented/available for this sensor")

    def _set_gyro_bandwidth(self):
        raise NotImplementedError("Method not implemented/available for this sensor")

    def _set_accelerometer_range(self):
        raise NotImplementedError("Method not implemented/available for this sensor")

    def _set_accelerometer_data_rate(self):
        raise NotImplementedError("Method not implemented/available for this sensor")

    def _set_accelerometer_bandwidth(self):
        raise NotImplementedError("Method not implemented/available for this sensor")


@dataclass
class GyroSpecs(ABC):
    """ 
    Gyro specs from the data sheet. Use for calibration & simulations 
    """
    zero_rate_offset: Union[float, None] = None  # Max zero rate offset [rad/s]
    output_noise_density: Union[float, None] = None  # Max output noise density [rad/s/sqrt(Hz)]
    bandwidth: Union[float, Non] = None  # Bandwidth [Hz]

    def __post_init__():
        if self.bandwidth is None:
            raise ValueError("Bandwidth must be set")

    def output_noise(self):
        """ Expected output noise per sample for a given bandwidth """
        return self.output_noise_density * np.sqrt(self.bandwidth)  # [rad/s/sqrt(Hz)]
