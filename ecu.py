"""
AEB Controller / ADAS Electronic Control Unit (ECU) Software.

In an SDV architecture, this software module operates as an autonomous ADAS microservice.
It consumes sensor signals from the Vehicle Signal Bus, executes safety algorithms,
and commands brake pressure via virtual actuator contracts.
"""

from typing import Tuple
from sdv_aeb.signals import AEBState, VehicleSignalBus


class AEBControllerECU:
    """
    Automatic Emergency Braking (AEB) Software Controller.
    Computes Time-To-Collision (TTC) and executes a multi-stage safety state machine.
    """

    def __init__(
        self,
        ttc_warning_threshold: float = 3.0,
        ttc_partial_threshold: float = 2.0,
        ttc_emergency_threshold: float = 1.2,
        partial_brake_demand: float = 0.4,
        min_safety_margin_m: float = 2.0,
    ):
        """
        :param ttc_warning_threshold: Time-to-collision threshold for visual/acoustic warning [s].
        :param ttc_partial_threshold: TTC threshold to initiate haptic / partial braking [s].
        :param ttc_emergency_threshold: TTC threshold to demand maximum emergency braking [s].
        :param partial_brake_demand: Normalized brake pressure for partial braking [0.0 - 1.0].
        :param min_safety_margin_m: Static fallback distance threshold [m].
        """
        self.ttc_warning_threshold = ttc_warning_threshold
        self.ttc_partial_threshold = ttc_partial_threshold
        self.ttc_emergency_threshold = ttc_emergency_threshold
        self.partial_brake_demand = partial_brake_demand
        self.min_safety_margin_m = min_safety_margin_m

        # Current internal state
        self.current_state: AEBState = AEBState.NORMAL

    def compute_ttc(self, distance: float, relative_speed: float) -> float:
        """
        Calculates Time-To-Collision (TTC) using relative kinematic velocity:
            TTC = Distance / Relative_Speed
        
        If relative velocity is zero or negative (moving away), collision is impossible,
        so TTC returns infinity (inf).

        :param distance: Distance to obstacle [m].
        :param relative_speed: Closing velocity [m/s].
        :return: TTC in seconds.
        """
        # Avoid division by zero or negative closing velocity (target opening distance)
        if relative_speed <= 0.05 or distance <= 0:
            return float("inf") if distance > 0 else 0.0

        ttc = distance / relative_speed
        return max(0.0, ttc)

    def step(self, bus: VehicleSignalBus) -> Tuple[AEBState, float, bool]:
        """
        Executes one AEB decision cycle based on vehicle bus telemetry:
        1. Read measured distance and speed from HAL signals.
        2. Compute instantaneous Time-To-Collision (TTC).
        3. Evaluate state transitions: NORMAL -> WARNING -> PARTIAL -> EMERGENCY.
        4. Set warning flag and target brake pressure demand.

        :param bus: Reference to the shared VehicleSignalBus.
        :return: (new_state, target_brake_pressure [0.0 - 1.0], warning_alarm_active [bool])
        """
        dist = bus.sensor_measured_distance
        rel_speed = bus.sensor_relative_speed

        # Step 1: Calculate TTC
        ttc = self.compute_ttc(dist, rel_speed)
        bus.ttc = ttc

        # Step 2: Safety State Machine Evaluation
        # If vehicle is already stopped (speed ~ 0), return to NORMAL with no brake demand
        if bus.sensor_measured_speed <= 0.01:
            self.current_state = AEBState.NORMAL
            target_pressure = 0.0
            warning = False

        # State 4: EMERGENCY BRAKING
        # Triggered when TTC <= emergency threshold OR vehicle is dangerously close
        elif ttc <= self.ttc_emergency_threshold or dist <= self.min_safety_margin_m:
            self.current_state = AEBState.EMERGENCY_BRAKING
            target_pressure = 1.0  # 100% full brake capacity
            warning = True

        # State 3: PARTIAL BRAKING (Comfort / deceleration initiation)
        # Triggered when TTC is between emergency and partial thresholds
        elif ttc <= self.ttc_partial_threshold:
            self.current_state = AEBState.PARTIAL_BRAKING
            target_pressure = self.partial_brake_demand  # e.g., 40% pressure
            warning = True

        # State 2: WARNING (Acoustic/visual alert, driver takes over)
        # Triggered when TTC is between partial and warning thresholds
        elif ttc <= self.ttc_warning_threshold:
            self.current_state = AEBState.WARNING
            target_pressure = 0.0  # Alert driver without sudden deceleration
            warning = True

        # State 1: NORMAL (Safe distance, no intervention)
        else:
            self.current_state = AEBState.NORMAL
            target_pressure = 0.0
            warning = False

        # Update bus outputs
        bus.aeb_state = self.current_state
        bus.target_brake_pressure = target_pressure
        bus.warning_alarm = warning

        return self.current_state, target_pressure, warning
