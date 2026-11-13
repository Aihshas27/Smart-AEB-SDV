"""
Unit and Integration Tests for SDV AEB Simulation.
Verifies core physical equations, state machine transitions, and edge cases.
"""

import sys
from pathlib import Path

# Add current directory to path
sys.path.insert(0, str(Path(__file__).parent))

try:
    from sdv_aeb.simulator import AEBSimulator
    from sdv_aeb.signals import CollisionStatus, AEBState
except ModuleNotFoundError:
    from simulator import AEBSimulator
    from signals import CollisionStatus, AEBState


def test_safe_stop_scenario():
    """Test standard case: Car at 50 km/h with 45m distance on dry asphalt."""
    print("Running Test 1: Safe Stop Scenario (50 km/h, 45m)...")
    sim = AEBSimulator(
        initial_speed_kmh=50.0,
        initial_distance_m=45.0,
        friction_coeff_mu=0.85,
        brake_response_time_tau=0.15,
        ttc_warning=3.0,
        ttc_partial=2.0,
        ttc_emergency=1.2,
    )
    df, summary = sim.run_full()

    assert summary["collision_status"] == CollisionStatus.SAFE_STOP.value, f"Expected SAFE_STOP, got {summary['collision_status']}"
    assert summary["final_clearance_m"] > 0.0, f"Expected clearance > 0, got {summary['final_clearance_m']}"
    assert summary["impact_speed_kmh"] == 0.0, f"Expected 0 impact speed, got {summary['impact_speed_kmh']}"
    
    # Check that car did not move backwards
    assert (df["speed"] >= 0.0).all(), "Vehicle speed became negative!"
    
    # Check that AEB states progressed
    states_visited = df["aeb_state"].unique().tolist()
    print(f"  States visited: {states_visited}")
    assert AEBState.EMERGENCY_BRAKING.value in states_visited, "Emergency braking was not triggered!"
    print(f"  Result: Passed. Stopped in {summary['stopping_distance_m']}m with {summary['final_clearance_m']}m clearance.\n")


def test_unavoidable_collision_scenario():
    """Test edge case: Car at 100 km/h with only 15m distance (physically impossible to stop)."""
    print("Running Test 2: Unavoidable Collision Scenario (100 km/h, 15m)...")
    sim = AEBSimulator(
        initial_speed_kmh=100.0,
        initial_distance_m=15.0,
        friction_coeff_mu=0.85,
    )
    df, summary = sim.run_full()

    assert summary["collision_status"] == CollisionStatus.COLLISION.value, f"Expected COLLISION, got {summary['collision_status']}"
    assert summary["impact_speed_kmh"] > 0.0, f"Expected impact speed > 0, got {summary['impact_speed_kmh']}"
    print(f"  Result: Passed. Collision correctly detected with impact speed {summary['impact_speed_kmh']} km/h.\n")


def test_wet_road_reduced_friction():
    """Test wet road: Car at 60 km/h with 40m distance on wet asphalt (mu = 0.45)."""
    print("Running Test 3: Wet Road Friction Evaluation (60 km/h, 40m, mu=0.45)...")
    sim = AEBSimulator(
        initial_speed_kmh=60.0,
        initial_distance_m=40.0,
        friction_coeff_mu=0.45,
    )
    df, summary = sim.run_full()
    print(f"  Status: {summary['collision_status']}, Stopping Dist: {summary['stopping_distance_m']}m, Final Clearance: {summary['final_clearance_m']}m")
    assert not df.empty, "DataFrame should not be empty"
    print("  Result: Passed.\n")


def test_telemetry_columns():
    """Verify that all required metrics are present in telemetry DataFrame."""
    print("Running Test 4: Telemetry Column Schema Verification...")
    sim = AEBSimulator(initial_speed_kmh=40.0, initial_distance_m=30.0)
    df, _ = sim.run_full()

    required_cols = [
        "timestamp", "speed", "speed_kmh", "acceleration",
        "distance_to_pedestrian", "sensor_measured_speed", "sensor_measured_distance",
        "ttc", "aeb_state", "target_brake_pressure", "actual_brake_pressure",
        "warning_alarm", "collision_status", "pedestrian_x", "pedestrian_y"
    ]
    for col in required_cols:
        assert col in df.columns, f"Missing column {col} in telemetry!"
    print("  Result: All required telemetry columns verified.\n")


def test_walking_pedestrian_crossing():
    """Test pedestrian crossing kinematics: Car at 50 km/h, pedestrian crossing zebra."""
    print("Running Test 5: Walking Pedestrian Crossing Kinematics...")
    sim = AEBSimulator(
        initial_speed_kmh=50.0,
        initial_distance_m=45.0,
        pedestrian_behavior="WALKING",
        pedestrian_walking_speed_mps=1.2,
    )
    df, summary = sim.run_full()
    assert summary["collision_status"] == CollisionStatus.SAFE_STOP.value, f"Expected SAFE_STOP, got {summary['collision_status']}"
    assert "pedestrian_y" in df.columns, "Missing pedestrian_y column!"
    assert df["pedestrian_y"].iloc[0] > df["pedestrian_y"].iloc[-1], "Pedestrian lateral position did not advance across road!"
    print(f"  Pedestrian y translated from {df['pedestrian_y'].iloc[0]:.2f}m to {df['pedestrian_y'].iloc[-1]:.2f}m.")
    print(f"  Car safely stopped with {summary['final_clearance_m']:.2f}m clearance.")
    print("  Result: Passed.\n")


if __name__ == "__main__":
    print("==========================================")
    print("SDV AEB SYSTEM - CORE SIMULATION TESTS")
    print("==========================================\n")
    test_safe_stop_scenario()
    test_unavoidable_collision_scenario()
    test_wet_road_reduced_friction()
    test_telemetry_columns()
    test_walking_pedestrian_crossing()
    print("ALL TESTS PASSED SUCCESSFULLY! (5/5)")
