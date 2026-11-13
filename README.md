# Software-Based Automatic Emergency Braking (AEB) System for a Software-Defined Vehicle (SDV)

**Course**: Fundamentals of Software Defined Vehicle Systems  
**Project Type**: Pure Software Simulation (No Hardware / Microcontroller Dependencies)  
**Tech Stack**: Python 3, Streamlit, Plotly, NumPy, Pandas  

---

## 1. Executive Summary

This project implements a comprehensive, software-based **Automatic Emergency Braking (AEB)** system adhering to modern **Software-Defined Vehicle (SDV)** architectural paradigms.

The system simulates:
1. A vehicle traveling at a configurable initial speed along an asphalt roadway.
2. A stationary or crossing pedestrian hazard located ahead of the vehicle.
3. Virtual perception sensors (Distance & Speed sensors) operating under a Hardware Abstraction Layer (HAL).
4. Real-time calculation of **Time-To-Collision (TTC)**.
5. An ADAS AEB software controller (ECU) executing a 4-state safety machine (`NORMAL`, `WARNING`, `PARTIAL_BRAKING`, `EMERGENCY_BRAKING`).
6. A virtual electro-hydraulic brake actuator with realistic pressure rise time and lag ($\tau$).
7. 1D longitudinal vehicle dynamics incorporating tire-road friction ($\mu$), aerodynamic drag, and rolling resistance in SI units.
8. Safe stopping vs. collision detection without backwards rolling motion ($v \ge 0$).
9. A 4-tab interactive dashboard with **4 selectable camera views** (Top-Down, Side, Third-Person Chase, and Driver POV Cockpit), live telemetry gauges, synchronized Plotly charts, and an automated test suite runner.

---

## 2. Software-Defined Vehicle (SDV) Architectural Pillars

```
+-----------------------------------------------------------------------------+
|               Streamlit Multi-Camera Automotive HMI & Control Deck          |
+-----------------------------------------------------------------------------+
                                       ^
                                       | (Telemetry & State Stream)
+-----------------------------------------------------------------------------+
|                       ADAS AEB Software Service (ECU)                       |
|   - Time-To-Collision (TTC) Estimator: TTC = d / v_rel                      |
|   - 4-State Safety Decision Machine: NORMAL -> WARN -> PARTIAL -> EMERGENCY |
|   - Software-Defined Safety Thresholds (OTA Updatable)                      |
+-----------------------------------------------------------------------------+
              ^                                                 |
              | (VSS Signals)                                   | (Command Bus)
+-----------------------------------------------------------------------------+
|                 Centralized Vehicle Signal Bus (COVESA VSS)                 |
|   Vehicle.Speed | Vehicle.ObstacleDistance | Vehicle.TTC | Vehicle.AEBState |
+-----------------------------------------------------------------------------+
          ^                                                         |
          | (Sensor Readings)                                       v (Hydraulic Pressure)
+----------------------------------+                   +----------------------------------+
|   Virtual Sensor Layer (HAL)     |                   |      Virtual Actuator Layer      |
| - VirtualSpeedSensor (Wheel/IMU) |                   | - VirtualBrakeActuator (EHB Lag) |
| - VirtualDistanceSensor (Radar)  |                   |   dP/dt = (P_target - P) / tau   |
+----------------------------------+                   +----------------------------------+
                  ^                                                 |
                  |                                                 v
+-----------------------------------------------------------------------------------------+
|                          1D Longitudinal Vehicle Dynamics Model                         |
|           F_net = F_brake(mu, m, g) + F_drag + F_roll  |  a = - (F_net / m)             |
+-----------------------------------------------------------------------------------------+
```

1. **Hardware-Software Decoupling (HAL)**:
   - Sensors and actuators are abstracted into software interfaces (`VirtualSpeedSensor`, `VirtualDistanceSensor`, `VirtualBrakeActuator`).
   - The application logic operates on abstract signals rather than hardware-specific microcontroller registers.
2. **COVESA Vehicle Signal Specification (VSS)**:
   - Components exchange data through a standardized signal bus (`VehicleSignalBus`), modeling an automotive publish/subscribe signal broker.
3. **Over-The-Air (OTA) Reconfigurability**:
   - Safety thresholds ($T_{\text{warning}}$, $T_{\text{partial}}$, $T_{\text{emergency}}$) and braking demands can be tuned live without modifying physical hardware.
4. **Software-in-the-Loop (SiL) Virtual Validation**:
   - Enables safety verification and edge-case testing (e.g., icy road, high-speed collision limits) purely in software.

---

## 3. Four Interactive Dashboard Tabs

### 🚗 Tab 1: Live Vehicle Simulation & Multi-Camera Views
Provides a large, high-resolution simulation scene with an **embedded 60 FPS HTML5/SVG simulation player** and **4 selectable camera modes**:
1. **Top-Down External Aerial View**:
   - Complete two-lane asphalt roadway, dashed lane markers, road boundaries, and zebra crossing.
   - Aerial car with wheels, roof, windshields, active headlights, and **real-time red rear brake lights** that illuminate upon braking.
   - Animated human pedestrian walking with **bipedal limb gait cycle** across the zebra crossing.
   - Dynamic AEB radar perception corridor color-coded by ECU state, plus distance clearance ruler.
2. **Side External Profile View**:
   - Side elevation of the car with rotating alloy wheels whose spin speed is proportional to vehicle velocity.
   - Glowing brake calipers and taillights during deceleration.
   - Human pedestrian crossing the road with swinging arms and stride cycle.
3. **Third-Person Follow / Chase Camera View**:
   - Dynamic chase perspective positioned behind and slightly above the car following the vehicle down the lane.
   - Rear fascia with high-intensity LED brake light clusters that glow red upon deceleration.
   - Animated pedestrian walking ahead across the lane with real perspective depth.
4. **First-Person Driver / Cockpit POV View**:
   - Realistic interior cockpit view through the front windshield.
   - A-pillars, dashboard cowl, rearview mirror, steering wheel, and digital instrument cluster.
   - Visual/auditory chime during `WARNING` and a prominent flashing alert banner during `EMERGENCY_BRAKING`.
- **Zero-Reload Camera Switching**: Switching between the 4 camera perspectives occurs instantly without restarting or resetting the simulation.
- **Playback Controls**: Full interactive deck with Play, Pause, Reset, Replay, Step (+0.1s), Speed selector (0.5x, 1.0x, 2.0x), and Timeline Scrubber.

### 📊 Tab 2: Telemetry & Vehicle Instruments
- High-visibility automotive KPI instrument cards: Speed (km/h & m/s), Clearance (m), TTC (s), Brake Pressure (%), and Acceleration ($m/s^2$ & G-force).
- **4 Synchronized Plotly Charts**:
  1. *Vehicle Speed vs. Time [km/h]*
  2. *Distance to Pedestrian vs. Time [m]* (with impact boundary)
  3. *Time-To-Collision (TTC) vs. Time [s]* (with Warning, Partial, and Emergency threshold indicator lines)
  4. *Brake Pressure vs. Time [%]* (Target command vs. actual hydraulic response curve)
- State duration breakdown (% of time spent in each AEB state).
- Complete telemetry inspection table and **CSV Export** button.

### ⚡ Tab 3: SDV Architecture & Live Signal Bus
- Interactive system architecture diagram with animated signal paths and color-coded state highlights.
- Real-time COVESA VSS Signal Table: `Vehicle.Speed`, `Vehicle.ADAS.ObstacleDistance`, `Vehicle.ADAS.TTC`, `Vehicle.ADAS.AEBState`, `Vehicle.Chassis.Brake.PedalPosition`, `Vehicle.Chassis.Brake.ActualPressure`, `Vehicle.CurrentPosition`, `Vehicle.Safety.CollisionStatus`.
- Academic explanations on Hardware-Software Decoupling (HAL), VSS ontology, and Software-in-the-Loop (SiL) testing for college project vivas.

### 🧪 Tab 4: Scenarios, Safety Analytics & Test Suite
- **7 Selectable Scenarios**:
  1. *Standard Urban* (50 km/h, 45m, Dry Asphalt $\mu = 0.85$)
  2. *High-Speed Highway* (100 km/h, 75m, Dry Asphalt $\mu = 0.85$)
  3. *Wet Road* (50 km/h, 40m, Wet Asphalt $\mu = 0.50$)
  4. *Low-Friction / Icy Road* (40 km/h, 45m, Packed Snow/Ice $\mu = 0.20$)
  5. *Sudden Pedestrian Crossing Hazard* (45 km/h, 30m, Pedestrian Crossing at 4 km/h)
  6. *Unavoidable Collision* (80 km/h, 18m, Dry Asphalt $\mu = 0.85$)
  7. *Custom Scenario* (Configurable sliders for all physical and ADAS parameters)
- **Physics Validation Table**: Compares simulated stopping distance against textbook ideal minimum stopping distance ($d = \frac{v_0^2}{2 \mu g}$).
- **Automated Test Suite Runner**: Executes [`test_simulation.py`](file:///c:/Users/aihsh/Desktop/aihshas/test_simulation.py) directly from the browser and reports actual terminal output and pass/fail metrics.

---

## 4. Mathematical Foundations

### 4.1 Time-To-Collision (TTC)
$$\text{TTC} = \frac{d(t)}{v_{\text{rel}}(t)}$$
Where $d(t)$ is relative distance and $v_{\text{rel}}(t) = v_{\text{vehicle}}(t) - v_{\text{pedestrian}}(t)$ is closing velocity. If $v_{\text{rel}} \le 0.05\,\text{m/s}$, $\text{TTC} = \infty$.

### 4.2 AEB Decision State Machine
| State | Trigger Condition | Commanded Brake Pressure | Driver Alert | Action |
| :--- | :--- | :--- | :--- | :--- |
| **`NORMAL`** | $\text{TTC} > 3.0\,\text{s}$ | $0\%$ | Disabled | Free driving, driver in control. |
| **`WARNING`** | $2.0\,\text{s} < \text{TTC} \le 3.0\,\text{s}$ | $0\%$ | **ACTIVE** | Visual/auditory chime alerting driver. |
| **`PARTIAL_BRAKING`** | $1.2\,\text{s} < \text{TTC} \le 2.0\,\text{s}$ | $40\%$ | **ACTIVE** | Caliper pre-fill, haptic jerk deceleration. |
| **`EMERGENCY_BRAKING`** | $\text{TTC} \le 1.2\,\text{s}$ or $d \le 2.0\,\text{m}$ | $100\%$ | **ACTIVE** | Maximum autonomous stopping force. |

### 4.3 Electro-Hydraulic Actuator Lag (Brake-by-Wire)
$$\frac{dP}{dt} = \frac{P_{\text{target}} - P_{\text{actual}}}{\tau}$$
Discretized via Euler method with typical automotive response constant $\tau \approx 0.15\,\text{s}$.

### 4.4 Longitudinal Dynamics & Friction (SI Units)
$$\sum F = - (F_{\text{brake}} + F_{\text{drag}} + F_{\text{roll}}) = m \cdot a$$
- $F_{\text{brake}} = P_{\text{actual}} \cdot (\mu \cdot m \cdot g)$
- $F_{\text{drag}} = \frac{1}{2} \cdot \rho \cdot C_d \cdot A \cdot v(t)^2$
- $F_{\text{roll}} = C_{rr} \cdot m \cdot g$
- Non-reversing constraint: $v(t) \ge 0$.

---

## 5. How to Run

### Step 1: Install Dependencies
```bash
pip install -r requirements.txt
```

### Step 2: Run Automated Simulation Tests
```bash
python test_simulation.py
```
*Expected Output:*
```
==========================================
SDV AEB SYSTEM - CORE SIMULATION TESTS
==========================================
Test 1: Safe Stop Scenario (50 km/h, 45m)... Passed
Test 2: Unavoidable Collision Scenario (100 km/h, 15m)... Passed
Test 3: Wet Road Friction Evaluation (60 km/h, 40m, mu=0.45)... Passed
Test 4: Telemetry Column Schema Verification... Passed
Test 5: Walking Pedestrian Crossing Kinematics... Passed
ALL TESTS PASSED SUCCESSFULLY! (5/5)
```

### Step 3: Launch Streamlit Dashboard
```bash
streamlit run sdv_aeb/app.py
```
Open **`http://localhost:8501`** in your browser.
