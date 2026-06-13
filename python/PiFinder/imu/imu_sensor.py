"""
Base class to interface with different IMU sensors

Enalbes interfacing to different IMUs and running in different modes in an
uniform way. Some IMUs may not support all the methods.
"""

from abc import ABC


class ImuSensor(ABC):
    
    def __init__(self):
        pass

    @abstractmethod
    def configure(self):
        pass

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
class GyroSpecs:
    """ Gyro specs from the data sheet. Use for calibration & simulations """
    zero_rate_offset: Union[float, None] = None  # Max zero rate offset [rad/s]
    output_noise_density: Union[float, None] = None  # Max output noise density [rad/s/sqrt(Hz)]

    def output_noise(self, bandwidth: float):
        """ Expected output noise per sample for a given bandwidth [Hz] """
        return self.output_noise_density * np.sqrt(bandwidth)  # [rad/s/sqrt(Hz)]
