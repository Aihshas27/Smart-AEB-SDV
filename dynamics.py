"""
Vehicle Dynamics and Physics Simulation Engine.

Simulates 1D vehicle kinematics, aerodynamic drag, rolling resistance,
and tire-road contact friction using standard SI units.
Ensures physically sound constraints (e.g., brakes cannot pull car backwards).
"""

from typing import Tuple
from sdv_aeb.signals import CollisionStatus, VehicleSignalBus


class VehicleDynamics:
    """
    1D Longitudinal Vehicle Dynamics Model.
    Governed by Newton's Second Law: m * a = - (F_brake + F_drag + F_roll)
    """

    def __init__(
        self,
        mass_kg: float = 1500.0,
        friction_coeff_mu: float = 0.85,
        air_density_rho: float = 1.225,
        drag_coeff_cd: float = 0.30,
        frontal_area_m2: float = 2.2,
        rolling_res_crr: float = 0.015,
        gravity_g: float = 9.81,
    ):
        """
        :param mass_kg: Vehicle curb mass [kg].
        :param friction_coeff_mu: Tire-road peak friction coefficient mu (Dry: 0.85, Wet: 0.5, Ice: 0.15).
        :param air_density_rho: Ambient air density [kg/m^3].
        :param drag_coeff_cd: Aerodynamic drag coefficient [-].
        :param frontal_area_m2: Vehicle frontal cross-sectional area [m^2].
        :param rolling_res_crr: Tire rolling resistance coefficient [-].
        :param gravity_g: Acceleration due to gravity [m/s^2].
        """
        self.mass = mass_kg
        self.mu = friction_coeff_mu
        self.rho = air_density_rho
        self.cd = drag_coeff_cd
        self.frontal_area = frontal_area_m2
        self.crr = rolling_res_crr
        self.g = gravity_g

        # Internal state variables
        self.speed = 0.0                       # [m/s]
        self.position = 0.0                    # [m]
        self.acceleration = 0.0                # [m/s^2]
        self.initial_distance = 0.0            # [m]
        self.pedestrian_speed = 0.0            # [m/s] longitudinal
        self.pedestrian_position = 0.0         # [m] longitudinal position (alias for pedestrian_x)
        self.pedestrian_x = 0.0                # [m] longitudinal position
        self.pedestrian_y = 0.0                # [m] lateral position across road (0 = lane center)
        self.pedestrian_behavior = "STATIONARY" # "STATIONARY", "WALKING", "SUDDEN_CROSSING"
        self.pedestrian_walking_speed = 1.2    # [m/s] lateral crossing speed
        self.pedestrian_start_time = 0.0       # [s] crossing delay
        self.sim_time = 0.0                    # [s] elapsed simulation time

    def reset(
        self,
        initial_speed_mps: float,
        initial_pedestrian_dist_m: float,
        pedestrian_speed_mps: float = 0.0,
        pedestrian_behavior: str = "STATIONARY",
        pedestrian_walking_speed_mps: float = 1.2,
        pedestrian_start_time_s: float = 0.0,
    ) -> None:
        """Resets the dynamic state for a new simulation run."""
        self.speed = max(0.0, initial_speed_mps)
        self.position = 0.0
        self.acceleration = 0.0
        self.initial_distance = max(0.1, initial_pedestrian_dist_m)
        self.pedestrian_speed = pedestrian_speed_mps
        self.pedestrian_x = self.initial_distance
        self.pedestrian_position = self.initial_distance
        self.pedestrian_behavior = pedestrian_behavior.upper()
        self.pedestrian_walking_speed = max(0.2, pedestrian_walking_speed_mps)
        self.pedestrian_start_time = max(0.0, pedestrian_start_time_s)
        self.sim_time = 0.0

        # Set initial lateral position based on behavior
        if self.pedestrian_behavior == "STATIONARY":
            self.pedestrian_y = 0.0  # Standing in vehicle lane center
        elif self.pedestrian_behavior in ["WALKING", "WALKING_ACROSS_ROAD"]:
            self.pedestrian_y = 3.8  # Starts at left sidewalk/curb
        elif self.pedestrian_behavior in ["SUDDEN_CROSSING", "SUDDEN"]:
            self.pedestrian_y = 3.5  # Waits near the curb edge
        else:
            self.pedestrian_y = 0.0

    def compute_theoretical_stopping_distance(self, initial_speed_mps: float) -> float:
        """
        Computes ideal physical minimum stopping distance assuming instantaneous full braking:
            d_ideal = v_0^2 / (2 * mu * g)
        
        :param initial_speed_mps: Starting velocity [m/s].
        :return: Ideal stopping distance [m].
        """
        max_decel = self.mu * self.g
        return (initial_speed_mps ** 2) / (2.0 * max_decel)

    def step(self, actual_brake_pressure: float, dt: float) -> Tuple[float, float, float, CollisionStatus]:
        """
        Integrates longitudinal equations of motion over timestep dt.

        Equations:
        1. F_brake = actual_brake_pressure * (mu * m * g)
        2. F_drag = 0.5 * rho * Cd * A * v^2
        3. F_roll = Crr * m * g  (when v > 0)
        4. a = - (F_brake + F_drag + F_roll) / m
        5. v(t + dt) = max(0, v(t) + a * dt)
        6. x(t + dt) = x(t) + v_avg * dt

        :param actual_brake_pressure: Developed hydraulic pressure fraction [0.0 - 1.0].
        :param dt: Integration timestep [s].
        :return: (speed, position, distance_to_pedestrian, collision_status)
        """
        # If vehicle is already at rest, it must not roll backward due to brake forces
        if self.speed <= 0.001:
            self.speed = 0.0
            self.acceleration = 0.0
            self.sim_time += dt
            # Update pedestrian location
            self._update_pedestrian(dt)
            delta_x = max(0.0, self.pedestrian_x - self.position)
            rel_dist = (delta_x**2 + self.pedestrian_y**2)**0.5 if self.pedestrian_y != 0.0 else delta_x
            return self.speed, self.position, rel_dist, CollisionStatus.SAFE_STOP

        self.sim_time += dt

        # 1. Braking Force (limited by tire-road adhesion mu * N)
        f_normal = self.mass * self.g
        f_brake = actual_brake_pressure * (self.mu * f_normal)

        # 2. Aerodynamic Drag Force
        f_drag = 0.5 * self.rho * self.cd * self.frontal_area * (self.speed ** 2)

        # 3. Rolling Resistance Force
        f_roll = self.crr * f_normal

        # Total resistive force opposing motion
        total_resistive_force = f_brake + f_drag + f_roll

        # Newton's Second Law: a = F_net / m (negative because opposing forward velocity)
        self.acceleration = - (total_resistive_force / self.mass)

        # Numerical integration (Semi-implicit Euler)
        new_speed = self.speed + self.acceleration * dt

        # Prevent backward motion: brakes only bring speed to 0
        if new_speed < 0.0:
            # Vehicle stopped during this timestep
            dt_stop = -self.speed / self.acceleration if self.acceleration < 0 else dt
            dx = 0.5 * self.speed * dt_stop
            self.position += dx
            self.speed = 0.0
            self.acceleration = 0.0
        else:
            dx = 0.5 * (self.speed + new_speed) * dt
            self.position += dx
            self.speed = new_speed

        # Update pedestrian location
        self._update_pedestrian(dt)
        delta_x = self.pedestrian_x - self.position

        # Safety & Collision Boundary Evaluation
        if delta_x <= 0.0:
            # Check if pedestrian is laterally within vehicle bumper width (|y| <= 1.25m)
            if self.pedestrian_behavior == "STATIONARY" or abs(self.pedestrian_y) <= 1.25:
                status = CollisionStatus.COLLISION
                rel_dist = 0.0
            else:
                # Pedestrian has already safely crossed out of vehicle lane
                status = CollisionStatus.SAFE_STOP
                rel_dist = abs(self.pedestrian_y)
        elif self.speed <= 0.001:
            # Vehicle safely came to rest before pedestrian
            status = CollisionStatus.SAFE_STOP
            rel_dist = (delta_x**2 + self.pedestrian_y**2)**0.5 if self.pedestrian_y != 0.0 else delta_x
        else:
            status = CollisionStatus.RUNNING
            rel_dist = (delta_x**2 + self.pedestrian_y**2)**0.5 if self.pedestrian_y != 0.0 else delta_x

        return self.speed, self.position, rel_dist, status

    def _update_pedestrian(self, dt: float) -> None:
        """Updates longitudinal and lateral coordinates of the pedestrian."""
        self.pedestrian_x += self.pedestrian_speed * dt
        self.pedestrian_position = self.pedestrian_x

        if self.pedestrian_behavior == "STATIONARY":
            self.pedestrian_y = 0.0
        elif self.pedestrian_behavior in ["WALKING", "WALKING_ACROSS_ROAD"]:
            if self.sim_time >= self.pedestrian_start_time:
                self.pedestrian_y = max(-4.2, self.pedestrian_y - self.pedestrian_walking_speed * dt)
        elif self.pedestrian_behavior in ["SUDDEN_CROSSING", "SUDDEN"]:
            trigger_dist = 35.0
            if self.sim_time >= max(0.6, self.pedestrian_start_time) or (self.pedestrian_x - self.position) <= trigger_dist:
                crossing_spd = max(2.0, self.pedestrian_walking_speed * 1.5)
                self.pedestrian_y = max(-4.2, self.pedestrian_y - crossing_spd * dt)
