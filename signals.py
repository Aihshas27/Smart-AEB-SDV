"""
Software-Defined Vehicle (SDV) - Vehicle Signal Bus & Data Contracts.

In an SDV architecture, software components communicate via standardized
signals decoupled from physical hardware (inspired by COVESA Vehicle Signal Specification).
"""

from enum import Enum
from dataclasses import dataclass, field
from typing import Dict, Any


class AEBState(str, Enum):
    """AEB Operational State Machine States."""
    NORMAL = "NORMAL"
    WARNING = "WARNING"
    PARTIAL_BRAKING = "PARTIAL_BRAKING"
    EMERGENCY_BRAKING = "EMERGENCY_BRAKING"


class CollisionStatus(str, Enum):
    """Current safety / collision status of the vehicle."""
    RUNNING = "RUNNING"
    SAFE_STOP = "SAFE_STOP"
    COLLISION = "COLLISION"


@dataclass
class VehicleSignalBus:
    """
    Centralized Vehicle Signal Bus (VSS Model).
    Represents the shared software state of the vehicle at any simulation timestep.
    All physical units are strictly in SI (meters, seconds, m/s, %, etc.).
    """
    # Timestamp
    timestamp: float = 0.0                      # [s] Current simulation time

    # Vehicle Dynamics Signals
    speed: float = 0.0                          # [m/s] Actual vehicle velocity
    speed_kmh: float = 0.0                      # [km/h] Vehicle velocity for HMI display
    acceleration: float = 0.0                   # [m/s^2] Current vehicle acceleration
    distance_to_pedestrian: float = 0.0         # [m] Ground truth relative distance
    pedestrian_speed: float = 0.0               # [m/s] Speed of pedestrian (0 for stationary)
    pedestrian_x: float = 0.0                   # [m] Longitudinal coordinate of pedestrian
    pedestrian_y: float = 0.0                   # [m] Lateral coordinate of pedestrian across road (0 = lane center)
    pedestrian_behavior: str = "STATIONARY"     # Pedestrian behavior mode: STATIONARY, WALKING, SUDDEN_CROSSING

    # Virtual Sensor Signals (HAL layer output)
    sensor_measured_speed: float = 0.0          # [m/s] Measured by wheel speed/IMU
    sensor_measured_distance: float = 0.0       # [m] Measured by Front Radar/Camera
    sensor_relative_speed: float = 0.0          # [m/s] Relative approach velocity

    # ADAS / AEB ECU Output Signals
    ttc: float = float("inf")                   # [s] Calculated Time-To-Collision
    aeb_state: AEBState = AEBState.NORMAL       # Current AEB State
    target_brake_pressure: float = 0.0          # [0.0 to 1.0] Commanded brake pressure (0% - 100%)
    warning_alarm: bool = False                 # Acoustic / Visual driver alert trigger

    # Actuator State Signals
    actual_brake_pressure: float = 0.0          # [0.0 to 1.0] Actual hydraulic brake pressure

    # Outcome Status
    position: float = 0.0                       # [m] Longitudinal distance traveled from origin
    collision_status: CollisionStatus = CollisionStatus.RUNNING
    impact_speed: float = 0.0                   # [m/s] Speed at impact if collision occurs
    stopping_distance: float = 0.0              # [m] Total distance traveled before stopping
    final_clearance: float = 0.0                # [m] Remaining gap to pedestrian after stop

    def snapshot(self) -> Dict[str, Any]:
        """Returns a serializable dictionary snapshot for telemetry logging."""
        return {
            "timestamp": round(self.timestamp, 4),
            "speed": round(self.speed, 3),
            "speed_kmh": round(self.speed * 3.6, 2),
            "acceleration": round(self.acceleration, 3),
            "position": round(self.position, 3),
            "distance_to_pedestrian": round(self.distance_to_pedestrian, 3),
            "pedestrian_x": round(self.pedestrian_x, 3),
            "pedestrian_y": round(self.pedestrian_y, 3),
            "sensor_measured_speed": round(self.sensor_measured_speed, 3),
            "sensor_measured_distance": round(self.sensor_measured_distance, 3),
            "ttc": 999.0 if self.ttc == float("inf") or self.ttc > 999.0 else round(self.ttc, 2),
            "aeb_state": self.aeb_state.value,
            "target_brake_pressure": round(self.target_brake_pressure * 100.0, 1),
            "actual_brake_pressure": round(self.actual_brake_pressure * 100.0, 1),
            "warning_alarm": self.warning_alarm,
            "collision_status": self.collision_status.value,
            "impact_speed_kmh": round(self.impact_speed * 3.6, 2),
        }
