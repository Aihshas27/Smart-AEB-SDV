"""
Virtual Actuator Layer (Brake-by-Wire / Electro-Hydraulic Brake Actuator).

Simulates mechanical and hydraulic brake dynamics in software. Real vehicle
hydraulic lines take time to pressurize; this actuator models that physical rise time.
"""

from typing import Tuple


class VirtualBrakeActuator:
    """
    Simulates an Electro-Hydraulic Braking (EHB) / Brake-by-Wire unit.
    Converts electronic brake command (0.0 - 1.0) into actual clamping pressure,
    accounting for hydraulic response delay (first-order lag).
    """

    def __init__(self, response_time_tau: float = 0.15):
        """
        :param response_time_tau: Hydraulic time constant tau [s] (typical automotive value: 0.1s - 0.25s).
        """
        self.response_time_tau = max(0.01, response_time_tau)
        self.actual_pressure = 0.0

    def reset(self, initial_pressure: float = 0.0) -> None:
        """Resets the actuator pressure state."""
        self.actual_pressure = initial_pressure

    def update(self, target_pressure: float, dt: float) -> float:
        """
        Updates actual brake pressure via a 1st-order differential lag equation:
            dP/dt = (P_target - P_actual) / tau
            P_actual(t + dt) = P_actual(t) + (dt / tau) * (P_target - P_actual(t))

        :param target_pressure: Commanded normalized pressure [0.0 to 1.0].
        :param dt: Timestep delta [s].
        :return: Actual developed brake pressure [0.0 to 1.0].
        """
        # Clamp command to [0.0, 1.0]
        target_clamped = max(0.0, min(1.0, target_pressure))

        # Euler discretization of 1st-order hydraulic lag
        alpha = dt / self.response_time_tau
        alpha = min(1.0, alpha)  # Prevent numerical overshoot for large dt

        self.actual_pressure += alpha * (target_clamped - self.actual_pressure)

        # Ensure pressure remains in [0.0, 1.0]
        self.actual_pressure = max(0.0, min(1.0, self.actual_pressure))
        return self.actual_pressure
