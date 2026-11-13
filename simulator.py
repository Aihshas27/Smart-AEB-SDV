"""
Closed-Loop SDV AEB Simulation Orchestrator.

Manages the cyclic execution of virtual sensors, the AEB ECU software,
the virtual brake actuator, and the vehicle physics model.
Records high-fidelity telemetry at every timestep.
"""

from typing import Dict, Any, List, Tuple
import pandas as pd
import numpy as np

try:
    from sdv_aeb.signals import VehicleSignalBus, AEBState, CollisionStatus
    from sdv_aeb.sensors import VirtualSpeedSensor, VirtualDistanceSensor
    from sdv_aeb.ecu import AEBControllerECU
    from sdv_aeb.actuators import VirtualBrakeActuator
    from sdv_aeb.dynamics import VehicleDynamics
except ModuleNotFoundError:
    from signals import VehicleSignalBus, AEBState, CollisionStatus
    from sensors import VirtualSpeedSensor, VirtualDistanceSensor
    from ecu import AEBControllerECU
    from actuators import VirtualBrakeActuator
    from dynamics import VehicleDynamics


class AEBSimulator:
    """
    Simulates the closed-loop Software-Defined Vehicle AEB system.
    Runs at a fixed discrete frequency (default: 50 Hz, dt = 0.02s).
    """

    def __init__(
        self,
        initial_speed_kmh: float = 60.0,
        initial_distance_m: float = 50.0,
        pedestrian_speed_kmh: float = 0.0,
        friction_coeff_mu: float = 0.85,
        brake_response_time_tau: float = 0.15,
        ttc_warning: float = 3.0,
        ttc_partial: float = 2.0,
        ttc_emergency: float = 1.2,
        partial_brake_demand: float = 0.4,
        sensor_dist_noise: float = 0.0,
        sensor_speed_noise: float = 0.0,
        vehicle_mass_kg: float = 1500.0,
        pedestrian_behavior: str = "STATIONARY",
        pedestrian_walking_speed_mps: float = 1.2,
        pedestrian_start_time_s: float = 0.0,
        dt: float = 0.02,
        max_duration_s: float = 15.0,
    ):
        # Configuration
        self.dt = dt
        self.max_duration = max_duration_s
        self.initial_speed_mps = initial_speed_kmh / 3.6
        self.initial_distance_m = initial_distance_m
        self.pedestrian_speed_mps = pedestrian_speed_kmh / 3.6
        self.pedestrian_behavior = pedestrian_behavior
        self.pedestrian_walking_speed_mps = pedestrian_walking_speed_mps
        self.pedestrian_start_time_s = pedestrian_start_time_s

        # Software Defined Subsystems
        self.bus = VehicleSignalBus()
        self.speed_sensor = VirtualSpeedSensor(noise_std=sensor_speed_noise, update_rate_hz=1.0 / dt)
        self.distance_sensor = VirtualDistanceSensor(
            max_range=150.0, noise_std=sensor_dist_noise, update_rate_hz=1.0 / dt
        )
        self.ecu = AEBControllerECU(
            ttc_warning_threshold=ttc_warning,
            ttc_partial_threshold=ttc_partial,
            ttc_emergency_threshold=ttc_emergency,
            partial_brake_demand=partial_brake_demand,
        )
        self.actuator = VirtualBrakeActuator(response_time_tau=brake_response_time_tau)
        self.dynamics = VehicleDynamics(mass_kg=vehicle_mass_kg, friction_coeff_mu=friction_coeff_mu)

        # Simulation telemetry log
        self.telemetry_history: List[Dict[str, Any]] = []
        self.is_finished = False

        self.reset()

    def reset(self) -> None:
        """Resets the simulation to the initial conditions."""
        self.dynamics.reset(
            initial_speed_mps=self.initial_speed_mps,
            initial_pedestrian_dist_m=self.initial_distance_m,
            pedestrian_speed_mps=self.pedestrian_speed_mps,
            pedestrian_behavior=self.pedestrian_behavior,
            pedestrian_walking_speed_mps=self.pedestrian_walking_speed_mps,
            pedestrian_start_time_s=self.pedestrian_start_time_s,
        )
        self.actuator.reset(0.0)
        self.telemetry_history.clear()
        self.is_finished = False

        # Initialize bus
        self.bus.timestamp = 0.0
        self.bus.speed = self.dynamics.speed
        self.bus.speed_kmh = self.bus.speed * 3.6
        self.bus.pedestrian_x = self.dynamics.pedestrian_x
        self.bus.pedestrian_y = self.dynamics.pedestrian_y
        self.bus.pedestrian_behavior = self.pedestrian_behavior
        self.bus.distance_to_pedestrian = (self.initial_distance_m**2 + self.dynamics.pedestrian_y**2)**0.5 if self.dynamics.pedestrian_y != 0 else self.initial_distance_m
        self.bus.pedestrian_speed = self.pedestrian_speed_mps
        self.bus.sensor_measured_speed = self.initial_speed_mps
        self.bus.sensor_measured_distance = self.bus.distance_to_pedestrian
        self.bus.sensor_relative_speed = max(0.0, self.initial_speed_mps - self.pedestrian_speed_mps)
        self.bus.ttc = self.ecu.compute_ttc(self.bus.sensor_measured_distance, self.bus.sensor_relative_speed)
        self.bus.aeb_state = AEBState.NORMAL
        self.bus.target_brake_pressure = 0.0
        self.bus.actual_brake_pressure = 0.0
        self.bus.warning_alarm = False
        self.bus.collision_status = CollisionStatus.RUNNING
        self.bus.impact_speed = 0.0
        self.bus.position = 0.0
        self.bus.stopping_distance = 0.0
        self.bus.final_clearance = self.bus.distance_to_pedestrian

        # Log initial state
        self.telemetry_history.append(self.bus.snapshot())

    def step(self) -> Tuple[VehicleSignalBus, bool]:
        """
        Executes one discrete simulation step of length dt.
        Simulation Flow:
            Virtual Sensors -> Signal Bus -> AEB ECU -> Brake Actuator -> Dynamics -> Evaluation

        :return: (updated_bus, is_simulation_complete)
        """
        if self.is_finished:
            return self.bus, True

        current_time = self.bus.timestamp + self.dt
        self.bus.timestamp = current_time

        # 1. Virtual Sensor Layer: Measure environmental state
        meas_speed = self.speed_sensor.measure(self.dynamics.speed, current_time)
        meas_dist, rel_speed = self.distance_sensor.measure(
            true_distance=self.bus.distance_to_pedestrian,
            true_vehicle_speed=self.dynamics.speed,
            true_pedestrian_speed=self.dynamics.pedestrian_speed,
            current_time=current_time,
        )

        # If pedestrian has lateral movement, calculate closing speed along line-of-sight
        if self.dynamics.pedestrian_y != 0.0 and meas_dist > 0.1:
            delta_x = max(0.0, self.dynamics.pedestrian_x - self.dynamics.position)
            # Pedestrian moves in -y direction when crossing road
            vy_ped = -self.dynamics.pedestrian_walking_speed if self.dynamics.pedestrian_behavior in ["WALKING", "WALKING_ACROSS_ROAD", "SUDDEN_CROSSING"] else 0.0
            closing_spd = (delta_x * self.dynamics.speed - self.dynamics.pedestrian_y * vy_ped) / meas_dist
            rel_speed = max(0.0, closing_spd)

        # Publish sensor data to bus
        self.bus.sensor_measured_speed = meas_speed
        self.bus.sensor_measured_distance = meas_dist
        self.bus.sensor_relative_speed = rel_speed

        # 2. Software ADAS Layer: AEB Controller Decision Cycle
        state, target_p, warning = self.ecu.step(self.bus)

        # 3. Virtual Actuator Layer: Pressure buildup through hydraulic lag
        actual_p = self.actuator.update(target_p, self.dt)
        self.bus.actual_brake_pressure = actual_p

        # 4. Vehicle Dynamics Layer: Physical motion integration
        new_speed, new_pos, rel_dist, status = self.dynamics.step(actual_p, self.dt)

        # 5. Update vehicle dynamics on the bus
        self.bus.speed = new_speed
        self.bus.speed_kmh = new_speed * 3.6
        self.bus.acceleration = self.dynamics.acceleration
        self.bus.distance_to_pedestrian = rel_dist
        self.bus.pedestrian_x = self.dynamics.pedestrian_x
        self.bus.pedestrian_y = self.dynamics.pedestrian_y
        self.bus.position = new_pos
        self.bus.stopping_distance = new_pos
        self.bus.final_clearance = rel_dist

        # Check termination criteria
        if status == CollisionStatus.COLLISION:
            self.bus.collision_status = CollisionStatus.COLLISION
            self.bus.impact_speed = new_speed
            self.is_finished = True
        elif status == CollisionStatus.SAFE_STOP:
            self.bus.collision_status = CollisionStatus.SAFE_STOP
            self.bus.impact_speed = 0.0
            self.is_finished = True
        elif current_time >= self.max_duration:
            # Timeout reached
            self.bus.collision_status = CollisionStatus.SAFE_STOP if self.bus.speed <= 0.01 else CollisionStatus.RUNNING
            self.is_finished = True

        # Log telemetry snapshot
        self.telemetry_history.append(self.bus.snapshot())

        return self.bus, self.is_finished

    def run_full(self) -> Tuple[pd.DataFrame, Dict[str, Any]]:
        """
        Runs the complete simulation until completion or timeout.
        
        :return: (telemetry_dataframe, summary_results_dictionary)
        """
        self.reset()
        while not self.is_finished:
            self.step()

        df = pd.DataFrame(self.telemetry_history)
        theoretical_stop_dist = self.dynamics.compute_theoretical_stopping_distance(self.initial_speed_mps)

        summary = {
            "initial_speed_kmh": round(self.initial_speed_mps * 3.6, 1),
            "initial_distance_m": round(self.initial_distance_m, 1),
            "collision_status": self.bus.collision_status.value,
            "impact_speed_kmh": round(self.bus.impact_speed * 3.6, 2),
            "stopping_distance_m": round(self.bus.stopping_distance, 2),
            "final_clearance_m": round(self.bus.final_clearance, 2),
            "theoretical_ideal_stopping_distance_m": round(theoretical_stop_dist, 2),
            "simulation_duration_s": round(self.bus.timestamp, 2),
            "max_deceleration_mps2": round(abs(df["acceleration"].min()), 2),
            "peak_brake_pressure_pct": round(df["actual_brake_pressure"].max(), 1),
        }
        return df, summary
