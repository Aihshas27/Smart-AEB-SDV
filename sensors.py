"""
Virtual Sensor Layer (Hardware Abstraction Layer - HAL).

Simulates vehicle perception hardware (Wheel Speed Sensor and Forward Radar/LiDAR)
entirely in software. In an SDV, the application logic interacts with these
abstract sensor interfaces rather than low-level physical bus registers.
"""

import numpy as np
from typing import Tuple


class VirtualSpeedSensor:
    """
    Simulates a digital Wheel Speed Sensor / IMU.
    Measures longitudinal vehicle velocity in SI units (m/s).
    """

    def __init__(self, noise_std: float = 0.0, update_rate_hz: float = 50.0):
        """
        :param noise_std: Standard deviation of Gaussian sensor measurement noise [m/s].
        :param update_rate_hz: Sensor sampling frequency [Hz].
        """
        self.noise_std = noise_std
        self.sample_period = 1.0 / update_rate_hz
        self.last_update_time = -1.0
        self._last_reading = 0.0

    def measure(self, true_speed: float, current_time: float) -> float:
        """
        Measures vehicle speed, applying sampling rate and sensor noise.
        
        :param true_speed: Ground truth speed [m/s].
        :param current_time: Current simulation time [s].
        :return: Measured speed [m/s] (clamped to >= 0).
        """
        if current_time - self.last_update_time >= self.sample_period or self.last_update_time < 0:
            noise = np.random.normal(0, self.noise_std) if self.noise_std > 0 else 0.0
            self._last_reading = max(0.0, true_speed + noise)
            self.last_update_time = current_time

        return self._last_reading


class VirtualDistanceSensor:
    """
    Simulates an ADAS Forward-Looking Radar / LiDAR / Camera sensor.
    Detects front obstacle (pedestrian) distance and relative velocity.
    """

    def __init__(
        self,
        max_range: float = 150.0,
        noise_std: float = 0.0,
        update_rate_hz: float = 50.0,
    ):
        """
        :param max_range: Maximum detection range of sensor [m].
        :param noise_std: Gaussian measurement noise on distance [m].
        :param update_rate_hz: Sensor refresh rate [Hz].
        """
        self.max_range = max_range
        self.noise_std = noise_std
        self.sample_period = 1.0 / update_rate_hz
        self.last_update_time = -1.0
        self._last_dist_reading = max_range
        self._last_rel_speed_reading = 0.0

    def measure(
        self,
        true_distance: float,
        true_vehicle_speed: float,
        true_pedestrian_speed: float,
        current_time: float,
    ) -> Tuple[float, float]:
        """
        Measures distance to obstacle and relative closing speed.

        :param true_distance: Ground truth Euclidean distance to obstacle [m].
        :param true_vehicle_speed: Host vehicle speed [m/s].
        :param true_pedestrian_speed: Pedestrian speed [m/s] (positive if moving away, 0 if static).
        :param current_time: Simulation timestamp [s].
        :return: (measured_distance [m], relative_closing_speed [m/s]).
        """
        if current_time - self.last_update_time >= self.sample_period or self.last_update_time < 0:
            # Check detection field of view / range limit
            if true_distance > self.max_range:
                measured_dist = self.max_range
                rel_speed = 0.0
            else:
                noise = np.random.normal(0, self.noise_std) if self.noise_std > 0 else 0.0
                measured_dist = max(0.0, true_distance + noise)
                # Relative closing speed = v_host - v_target
                rel_speed = max(0.0, true_vehicle_speed - true_pedestrian_speed)

            self._last_dist_reading = measured_dist
            self._last_rel_speed_reading = rel_speed
            self.last_update_time = current_time

        return self._last_dist_reading, self._last_rel_speed_reading
