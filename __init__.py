"""
Software-Defined Vehicle (SDV) Automatic Emergency Braking (AEB) Package.
"""

from sdv_aeb.signals import VehicleSignalBus, AEBState, CollisionStatus
from sdv_aeb.sensors import VirtualSpeedSensor, VirtualDistanceSensor
from sdv_aeb.ecu import AEBControllerECU
from sdv_aeb.actuators import VirtualBrakeActuator
from sdv_aeb.dynamics import VehicleDynamics
from sdv_aeb.simulator import AEBSimulator

from sdv_aeb.renderers import (
    render_top_down_view,
    render_side_view,
    render_third_person_view,
    render_driver_pov_view,
    render_architecture_diagram,
)

__all__ = [
    "VehicleSignalBus",
    "AEBState",
    "CollisionStatus",
    "VirtualSpeedSensor",
    "VirtualDistanceSensor",
    "AEBControllerECU",
    "VirtualBrakeActuator",
    "VehicleDynamics",
    "AEBSimulator",
    "render_top_down_view",
    "render_side_view",
    "render_third_person_view",
    "render_driver_pov_view",
    "render_architecture_diagram",
]
