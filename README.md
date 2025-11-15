# Leader-Follower Quadrotor Formation Control System

## Overview

This repository presents a comprehensive implementation of a **Leader-Follower quadrotor formation control system** utilizing PyBullet physics simulation. The system features a manually-controlled Leader drone and an autonomous Follower drone that maintains formation through cascaded PID control architecture. Built on the Crazyflie 2.0 platform model, the implementation includes complete state logging capabilities and automated generation of research-grade visualization plots.

---

## Table of Contents

1. [Features](#features)
2. [System Architecture](#system-architecture)
3. [Requirements](#requirements)
4. [Installation](#installation)
5. [Project Structure](#project-structure)
6. [Usage](#usage)
7. [Control Interface](#control-interface)
8. [Output Visualization](#output-visualization)
9. [Technical Implementation](#technical-implementation)
10. [Contributing](#contributing)
11. [License](#license)

---

## Features

### Formation Control
- High-fidelity quadrotor dynamics simulation using PyBullet physics engine
- Manual teleoperation of Leader drone via keyboard interface
- Autonomous formation tracking by Follower drone with configurable offset
- Stable tracking performance through cascaded DSL PID control loops
- Crazyflie 2.0 Plus (CF2P) platform model with realistic parameters
- Real-time velocity command interpretation and smooth trajectory generation
- Formation offset: Default 1.0m behind Leader in X-axis (configurable)

### Data Acquisition
The system automatically logs comprehensive state information for both vehicles:
- **Attitude**: Roll, Pitch, Yaw (Euler angles)
- **Position**: X, Y, Z coordinates in world frame
- **Velocity**: Linear velocities (Vx, Vy, Vz)
- **Tracking Error**: Real-time attitude error metrics (Leader - Follower)

### Visualization Suite
Upon simulation termination (Ctrl+C), the system automatically generates seven publication-quality figures:

1. **leader_follower_rpy.png** - Roll, Pitch, Yaw tracking comparison (3 subplots)
2. **leader_follower_xyz.png** - X, Y, Z position tracking comparison (3 subplots)
3. **leader_follower_velocity.png** - Velocity profile comparison: Vx, Vy, Vz (3 subplots)
4. **attitude_error_rpy.png** - Attitude error evolution over time (3 subplots)
5. **trajectory_3d.png** - Three-dimensional trajectory visualization with both drones
6. **combined_2x2_results.png** - Comprehensive 2×2 panel figure (Yaw, Position, Velocity, Error)
7. **combined_grouped_3.png** - Grouped 3-panel comparison (RPY, XYZ, Velocities on single axes)

All visualizations are exported at 300 DPI resolution to the `graphs/` directory using Matplotlib's Agg backend (headless rendering).

---

## System Architecture

The control system implements a hierarchical DSL (Distributed Sense and Learning) PID architecture:

```
┌─────────────────────────────────────────────┐
│         Leader Drone (Manual Control)       │
│   ┌─────────────────────────────────────┐   │
│   │  Keyboard Input → Velocity Commands │   │
│   │  Target Position Update (Carrot)    │   │
│   └─────────────────────────────────────┘   │
│                    ↓                         │
│   ┌─────────────────────────────────────┐   │
│   │  DSL PID Position Controller        │   │
│   │  - Position Error Calculation       │   │
│   │  - Velocity Error Calculation       │   │
│   │  - Integral Anti-windup             │   │
│   │  Output: Thrust + Desired Attitude  │   │
│   └─────────────────────────────────────┘   │
│                    ↓                         │
│   ┌─────────────────────────────────────┐   │
│   │  DSL PID Attitude Controller        │   │
│   │  - Rotation Matrix Error            │   │
│   │  - Angular Rate Damping             │   │
│   │  - Safety Tilt Limiting (±0.5 rad)  │   │
│   │  Output: Torque Commands            │   │
│   └─────────────────────────────────────┘   │
│                    ↓                         │
│   ┌─────────────────────────────────────┐   │
│   │  Motor Mixer (CF2P Configuration)   │   │
│   │  PWM → RPM Conversion               │   │
│   │  Output: 4 Motor RPM Commands       │   │
│   └─────────────────────────────────────┘   │
└─────────────────────────────────────────────┘
                     │
                     ↓ Leader Position Feedback
┌─────────────────────────────────────────────┐
│       Follower Drone (Autonomous)           │
│   ┌─────────────────────────────────────┐   │
│   │  Target Calculation:                │   │
│   │  follower_target = leader_pos +     │   │
│   │                    FOLLOW_OFFSET    │   │
│   │  Velocity Matching: leader_vel      │   │
│   └─────────────────────────────────────┘   │
│                    ↓                         │
│            [Same Control Stack]             │
│   Position PID → Attitude PID → Mixer       │
└─────────────────────────────────────────────┘
```

### Control Loop Hierarchy
- **Simulation Frequency**: 240 Hz (SIM_HZ)
- **Control Frequency**: 48 Hz (CTRL_HZ)
- **Control Timestep**: 0.0208 seconds
- **Physics Steps per Control**: 5 steps (SIM_STEPS)

---

## Requirements

### Software Dependencies
- Python 3.7 or higher
- PyBullet (physics simulation)
- NumPy (numerical computing)
- Matplotlib (visualization)
- SciPy (scientific computing)

### Hardware Requirements
- CPU: Modern multi-core processor recommended
- RAM: Minimum 4GB
- Display: OpenGL-compatible graphics for real-time visualization

---

## Installation

### Step 1: Clone the Repository
```bash
git clone <repository-url>
cd <repository-directory>
```

### Step 2: Install Dependencies
```bash
pip install pybullet numpy matplotlib scipy
```

Alternatively, use the requirements file (if provided):
```bash
pip install -r requirements.txt
```

### Step 3: Verify Installation
```bash
python3 --version
python3 -c "import pybullet, numpy, matplotlib, scipy; print('All dependencies installed successfully')"
```

---

## Project Structure

```
.
├── drone_follower.py       # Main simulation controller and orchestrator
├── DSLPIDControl.py        # DSL PID controller implementation (cascaded control)
├── BaseControl.py          # Base control class with URDF parameter loading
├── enums.py                # DroneModel enumeration definitions
├── cf2p.urdf               # Crazyflie 2.0 Plus URDF model file
├── cf2x.urdf               # Crazyflie 2.0 X configuration URDF (optional)
├── graphs/                 # Auto-generated visualization output directory
│   ├── leader_follower_rpy.png
│   ├── leader_follower_xyz.png
│   ├── leader_follower_velocity.png
│   ├── attitude_error_rpy.png
│   ├── trajectory_3d.png
│   ├── combined_2x2_results.png
│   └── combined_grouped_3.png
├── README.md               # Project documentation (this file)
└── requirements.txt        # Python package dependencies (optional)
```

---

## Usage

### Starting the Simulation

Execute the main script to launch the simulation environment:

```bash
python3 drone_follower.py
```

The PyBullet GUI will open, displaying both the Leader and Follower drones on a ground plane. The simulation runs in real-time with physics updates at the configured frequency.

### Terminating the Simulation

Press `Ctrl+C` in the terminal to gracefully terminate the simulation. This action triggers:
1. Automatic data processing
2. Generation of all visualization plots
3. Export of figures to the `graphs/` directory
4. Clean shutdown of the physics engine

---

## Control Interface

### Keyboard Commands

| Key | Function | Description |
|-----|----------|-------------|
| `R` | Forward | Increase velocity in positive X direction |
| `F` | Backward | Increase velocity in negative X direction |
| `D` | Left | Increase velocity in negative Y direction |
| `G` | Right | Increase velocity in positive Y direction |
| `↑` | Ascend | Increase altitude (positive Z) |
| `↓` | Descend | Decrease altitude (negative Z) |
| `Ctrl+C` | Stop | Terminate simulation and generate plots |

**Note**: All commands are applied as velocity setpoints to the Leader drone's position controller. The Follower autonomously maintains formation relative to the Leader's position.

---

## Output Visualization

### Generated Figures

Upon simulation termination, seven figures are automatically generated and saved to the `graphs/` directory:

1. **leader_follower_rpy.png**
   - Time-series comparison of Roll, Pitch, Yaw for both drones
   
2. **leader_follower_xyz.png**
   - Time-series comparison of X, Y, Z positions for both drones

3. **leader_follower_velocity.png**
   - Time-series comparison of linear velocities (Vx, Vy, Vz)

4. **attitude_error_rpy.png**
   - Evolution of attitude tracking error over time

5. **trajectory_3d.png**
   - Three-dimensional trajectory visualization

6. **combined_2x2_results.png**
   - Comprehensive 2×2 panel figure for presentations

7. **combined_grouped_3.png**
   - Grouped comparison figure (RPY + XYZ + Velocities)

### Figure Specifications
- **Resolution**: 300 DPI (publication quality)
- **Format**: PNG with transparent backgrounds
- **Backend**: Matplotlib Agg (headless rendering)

---

## Technical Implementation

### DSL PID Control Algorithm

The system implements a Distributed Sense and Learning (DSL) PID controller with cascaded loops and safety features.

#### Position Control Loop (Outer Loop)
```python
Input: current_position, target_position, current_velocity, target_velocity
Process:
  1. Calculate position error: e_pos = target - current
  2. Calculate velocity error: e_vel = target_vel - current_vel
  3. Update integral with anti-windup:
     - XY integral limits: [-2.0, 2.0]
     - Z integral limits: [-0.15, 0.15]
  4. Compute thrust vector:
     thrust_vec = Kp·e_pos + Ki·∫e_pos + Kd·e_vel + [0, 0, g]
  5. Calculate scalar thrust and convert to PWM
  6. Compute desired orientation from thrust vector
  7. Apply safety tilt limiting (max ±0.5 rad for roll/pitch)
Output: Thrust magnitude, Desired RPY angles
```

**Position PID Gains (Tuned for Stability)**:
- `P_COEFF_FOR`: [0.4, 0.4, 1.25] - Balanced position hold
- `I_COEFF_FOR`: [0.05, 0.05, 0.05] - Minimal integral action
- `D_COEFF_FOR`: [0.2, 0.2, 0.5] - Damping to prevent overshoot

#### Attitude Control Loop (Inner Loop)
```python
Input: current_quaternion, desired_euler_angles, target_rpy_rates
Process:
  1. Convert to rotation matrices
  2. Calculate rotation matrix error:
     R_err = R_target^T·R_current - R_current^T·R_target
  3. Extract rotation error vector from skew-symmetric matrix
  4. Calculate angular rate error using finite differences
  5. Update integral with anti-windup:
     - Roll/Pitch integral limits: [-1.0, 1.0]
     - Yaw integral limits: [-2.0, 2.0]
  6. Compute torque commands:
     τ = -Kp·rot_err + Kd·rate_err + Ki·∫rot_err
  7. Clip torques to [-3200, 3200]
Output: Motor torque commands
```

**Attitude PID Gains**:
- `P_COEFF_TOR`: [70000.0, 70000.0, 60000.0]
- `I_COEFF_TOR`: [0.0, 0.0, 500.0] - Only yaw integral
- `D_COEFF_TOR`: [20000.0, 20000.0, 12000.0]

#### Motor Mixing (Crazyflie 2.0 Plus Configuration)
```python
CF2P Mixer Matrix:
    [[ 0, -1, -1],    # Motor 0 (Front)
     [+1,  0, +1],    # Motor 1 (Right)
     [ 0, +1, -1],    # Motor 2 (Back)
     [-1,  0, +1]]    # Motor 3 (Left)

Process:
  1. Compute PWM commands: PWM = thrust + Mixer·torques
  2. Clip to range: [20000, 65535]
  3. Convert to RPM: ω = 0.2685·PWM + 4070.3
  4. Calculate thrust per motor: F = kf·ω²
Output: Four motor RPM values [ω₀, ω₁, ω₂, ω₃]
```

### URDF Parameter Loading

The `BaseControl` class dynamically loads physical parameters from URDF files:

```python
Loaded Parameters:
  - m: Drone mass
  - kf: Thrust coefficient
  - km: Moment coefficient
  - ixx, iyy, izz: Moments of inertia
  - arm: Distance from center to motor
  - Additional parameters: drag coefficients, propeller radius, etc.
```

### Formation Control Logic

**Leader Drone**:
```python
# Velocity command integration
target_pos += velocity_command * dt
target_pos[2] = max(target_pos[2], 0.05)  # Ground safety

# PID Control
rpm = controller.computeControl(
    cur_pos=leader_position,
    cur_quat=leader_quaternion,
    cur_vel=leader_velocity,
    target_pos=target_position,
    target_vel=velocity_command
)
```

**Follower Drone**:
```python
# Target calculation
FOLLOW_OFFSET = [-1.0, 0.0, 0.0]  # 1m behind in X
follower_target = leader_actual_position + FOLLOW_OFFSET
follower_target[2] = leader_target_altitude

# Velocity matching for smooth tracking
rpm = controller.computeControl(
    cur_pos=follower_position,
    cur_quat=follower_quaternion,
    cur_vel=follower_velocity,
    target_pos=follower_target,
    target_vel=leader_actual_velocity  # Match leader's velocity
)
```

---

## Simulation Parameters

### Physics Configuration
- **Physics Engine**: PyBullet with realistic rigid-body dynamics
- **Physics Timestep**: 1/240 seconds (240 Hz)
- **Control Update Rate**: 48 Hz (CTRL_HZ)
- **Control Timestep**: 0.0208 seconds (1/48)
- **Simulation Steps per Control**: 5 (SIM_STEPS = SIM_HZ / CTRL_HZ)
- **Gravity**: -9.81 m/s² (Z-axis)

### Drone Model
- **Platform**: Crazyflie 2.0 Plus (CF2P)
- **Configuration**: Plus (+) motor layout
- **URDF File**: `cf2p.urdf`
- **Visual Differentiation**: Follower has reddish color [1.0, 0.5, 0.5, 1.0]

### Motion Parameters
- **Maximum Speed**: 1.5 m/s (MOVE_SPEED) for XY motion
- **Climb Speed**: 1.0 m/s (CLIMB_SPEED) for Z motion
- **Formation Offset**: [-1.0, 0.0, 0.0] meters (1m behind Leader)
- **Minimum Altitude**: 0.05 meters (ground safety limit)
- **Maximum Tilt Angle**: ±0.5 radians (±28.6°) for roll and pitch

### Motor and Actuator Parameters
- **PWM to RPM Scale**: 0.2685
- **PWM to RPM Constant**: 4070.3
- **Minimum PWM**: 20000
- **Maximum PWM**: 65535
- **Torque Limits**: [-3200, 3200] for all axes

### Integral Anti-windup Limits
- **Position Integral (XY)**: [-2.0, 2.0] meters
- **Position Integral (Z)**: [-0.15, 0.15] meters
- **Attitude Integral (Roll/Pitch)**: [-1.0, 1.0] radians
- **Attitude Integral (Yaw)**: [-2.0, 2.0] radians

---

## Code Structure and Key Classes

### Main Simulation (`drone_follower.py`)
- **`setup_simulation(drone_urdf_path)`**: Initializes PyBullet environment, loads plane and two drones
- **`run_manual_control(drone1, drone2)`**: Main control loop with keyboard input processing, PID execution, and data logging
- **Visualization Functions**: Automatic plot generation on Ctrl+C interrupt

### DSL PID Controller (`DSLPIDControl.py`)
```python
class DSLPIDControl(BaseControl):
    def __init__(self, drone_model: DroneModel, g: float=9.8)
    def reset()
    def computeControl(control_timestep, cur_pos, cur_quat, cur_vel, 
                       cur_ang_vel, target_pos, target_rpy, 
                       target_vel, target_rpy_rates)
    def _dslPIDPositionControl()  # Outer loop
    def _dslPIDAttitudeControl()  # Inner loop
```

**Features**:
- Cascaded position and attitude control
- Integral anti-windup protection
- Safety tilt limiting
- Rotation matrix-based attitude error computation
- PWM to RPM conversion with Crazyflie-specific parameters

### Base Controller (`BaseControl.py`)
```python
class BaseControl(object):
    def __init__(self, drone_model: DroneModel, g: float=9.8)
    def reset()
    def computeControl()  # Abstract method
    def computeControlFromState()
    def setPIDCoefficients()
    def _getURDFParameter(parameter_name: str)
```

**Features**:
- URDF parameter extraction (mass, inertia, thrust coefficients)
- XML parsing for dynamic parameter loading
- Base class for all controller implementations
- Standardized control interface

### Enumerations (`enums.py`)
```python
class DroneModel(Enum):
    CF2X = "cf2x"  # Crazyflie 2.0 X configuration
    CF2P = "cf2p"  # Crazyflie 2.0 Plus configuration
```

### Data Logging
The system logs the following data arrays during simulation:
- `log_time`: Timestamp array
- `log_leader_rpy`: Leader Roll-Pitch-Yaw
- `log_follower_rpy`: Follower Roll-Pitch-Yaw
- `log_leader_pos`: Leader XYZ positions
- `log_follower_pos`: Follower XYZ positions
- `log_leader_vel`: Leader linear velocities
- `log_follower_vel`: Follower linear velocities

All arrays are converted to NumPy arrays for efficient processing and visualization.

Contributions to improve the system are welcome. Please follow these guidelines:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improvement`)
3. Commit your changes with clear messages
4. Push to your branch (`git push origin feature/improvement`)
5. Open a Pull Request with a detailed description

### Areas for Enhancement
- **Multi-follower support**: Extend to N-follower formations
- **Formation patterns**: Implement V-formation, line, circular, and custom geometries
- **Advanced control algorithms**: Model Predictive Control (MPC), LQR, adaptive control
- **Obstacle avoidance**: Real-time path planning with collision detection
- **Real-time parameter tuning**: GUI for live PID gain adjustment
- **Communication delays**: Simulate realistic network latency
- **Sensor noise modeling**: Add realistic IMU and position estimation errors
- **Wind disturbances**: External force modeling for robustness testing
- **Battery modeling**: Power consumption and flight time constraints
- **Waypoint navigation**: Predefined trajectory following for both drones

---

## License

This project is distributed under the MIT License. See `LICENSE` file for details.

---

## Citation

If you use this code in your research or academic work, please cite:

```bibtex
@software{leader_follower_quadrotor,
  title = {Leader-Follower Quadrotor Formation Control System},
  author = {[Your Name/Organization]},
  year = {2025},
  url = {[Repository URL]}
}
```

---

## Acknowledgments

- PyBullet physics engine by Erwin Coumans and the Bullet Physics team
- Crazyflie 2.0 model by Bitcraze AB
- Scientific Python ecosystem (NumPy, SciPy, Matplotlib)

---

## Contact

For questions, issues, or collaboration opportunities:
- **Issues**: Use the GitHub issue tracker
- **Email**: [Your contact email]
- **Documentation**: [Link to extended documentation if available]

---

## References

1. **PyBullet Physics Simulation**: https://pybullet.org/
2. **Crazyflie Platform Documentation**: https://www.bitcraze.io/documentation/
3. **PID Control Theory**: Åström, K. J., & Hägglund, T. (2006). *Advanced PID Control*. ISA-The Instrumentation, Systems, and Automation Society.
4. **Quadrotor Control**: Mahony, R., Kumar, V., & Corke, P. (2012). "Multirotor Aerial Vehicles: Modeling, Estimation, and Control of Quadrotor." *IEEE Robotics & Automation Magazine*, 19(3), 20-32.
5. **Formation Control**: Beard, R. W., Lawton, J., & Hadaegh, F. Y. (2001). "A coordination architecture for spacecraft formation control." *IEEE Transactions on Control Systems Technology*, 9(6), 777-790.
6. **Rotation Matrix Error**: Bullo, F., & Murray, R. M. (1999). "Tracking for fully actuated mechanical systems: a geometric framework." *Automatica*, 35(1), 17-34.

---

**Last Updated**: November 2025

**Status**: Active Development

---

*If this project was helpful for your work, please consider starring the repository and sharing it with the robotics community!*