# EKF Robot Localization Simulator

A C++ simulation framework for learning and implementing an **Extended Kalman Filter (EKF)** for 2D mobile robot localization with multi-sensor fusion.

## Overview

This project simulates a **differential drive robot** navigating through an indoor environment. The robot's position and heading are estimated by fusing data from three sensor types using an EKF:

- **Wheel Encoders** — Left/right wheel velocities with noise and systematic drift (prediction input)
- **IMU Gyroscope** — Angular velocity with bias and noise (prediction input)
- **Range-Bearing Sensor** — Range and bearing measurements to known landmarks (update step)

The simulation provides real-time SDL2 visualization showing the robot, ground truth path, EKF estimated path, covariance ellipses, landmarks, and error metrics.

## State Vector & EKF Mathematics

**State Vector:** `x = [px, py, theta]` (position + heading)

**Prediction Step (Odometry + IMU):**
```
px_new    = px + v * cos(theta) * dt
py_new    = py + v * sin(theta) * dt
theta_new = theta + omega * dt

Jacobian F:
[1  0  -v*sin(theta)*dt]
[0  1   v*cos(theta)*dt]
[0  0   1              ]
```

**Update Step (Range-Bearing to Landmarks):**
```
z = [range, bearing]
range_hat   = sqrt(dx^2 + dy^2)
bearing_hat = atan2(dy, dx) - theta

Jacobian H:
[-dx/r    -dy/r     0]
[ dy/r^2  -dx/r^2  -1]
```

## Project Structure

```
src/
├── main.cpp                         # Entry point, SDL2 init, 8 simulation profiles
├── robot.h                          # RobotState, DifferentialDriveMotion, MotionCommands, Robot
├── display.h / display.cpp          # SDL2 rendering pipeline
├── landmarks.h / landmarks.cpp      # Indoor environment with walls and landmark pillars
├── sensors.h / sensors.cpp          # WheelEncoder, IMU, RangeBearing sensors
├── kalmanfilter.h                   # KalmanFilterBase + KalmanFilter interface
├── kalmanfilter.cpp                 # Active EKF implementation
├── kalmanfilter_ekf_student.cpp     # Student template with TODO sections
├── kalmanfilter_ekf_answer.cpp      # Complete reference solution
├── simulation.h / simulation.cpp    # SimulationParams + update/render loop
└── utils.h / utils.cpp              # wrapAngle, RMSE, ellipse/circle generation
```

## Setup

### Prerequisites
```bash
sudo apt install libeigen3-dev libsdl2-dev libsdl2-ttf-dev
```

### Build & Run
```bash
mkdir build && cd build
cmake ../
make
./EKFRobotLocalization
```

## How to Use

The project has a **student template** and a **reference answer** file:

1. Study the EKF mathematics above
2. Copy `kalmanfilter_ekf_student.cpp` to `kalmanfilter.cpp`
3. Implement the prediction and update steps (marked with `ENTER YOUR CODE HERE`)
4. Build and test with simulation profiles 1-8
5. Compare your solution with `kalmanfilter_ekf_answer.cpp`

## Simulation Profiles

Press keys `1`-`8` during simulation:

| Key | Profile | Description |
|-----|---------|-------------|
| 1 | Straight Line | Simple forward motion, all sensors |
| 2 | Square Path | Tests turning behavior |
| 3 | Waypoint Navigation | Multi-room path, all sensors |
| 4 | Dead Reckoning Only | No landmarks — demonstrates drift |
| 5 | High Noise | 4x sensor noise on all channels |
| 6 | IMU Bias + Encoder Drift | Systematic sensor errors |
| 7 | Limited Range + FOV | 5m range, 270 degree field of view |
| 8 | CAPSTONE | All challenges combined |

## Controls

| Key | Action |
|-----|--------|
| `1`-`8` | Load simulation profile |
| `r` | Reset current simulation |
| `Space` | Pause / Resume |
| `]` / `[` | Increase / Decrease speed multiplier |
| `+` / `-` (numpad) | Zoom in / out |
| `Esc` | Quit |

## Environment

The simulation takes place in a 20m x 15m indoor environment with:
- **Walls** — Gray lines defining rooms and corridors
- **25 Landmarks** — Yellow diamond markers at known positions (pillars/columns)
- **4 Rooms** — Connected by a central corridor

## Exercises

### Exercise 1: EKF Prediction Step
Implement the state prediction using odometry velocity and IMU angular velocity. Compute the Jacobian F and process noise Q. Test with Profile 1.

### Exercise 2: EKF Update Step
Implement the range-bearing measurement update. Compute predicted measurement, Jacobian H, innovation, and Kalman gain. Test with Profiles 1-3.

### Exercise 3: Filter Initialization
Implement filter initialization from the first landmark measurement. Test with Profiles 1-2.

### Exercise 4: Robustness Testing
Test your filter with challenging Profiles 5-7. Tune noise parameters for best performance.

### Capstone
Achieve the best estimation performance on Profile 8 with all challenges combined.

## Dependencies

- [Eigen3](https://eigen.tuxfamily.org/) — Linear algebra
- [SDL2](https://www.libsdl.org/) — 2D graphics
- [SDL2_ttf](https://github.com/libsdl-org/SDL_ttf) — Font rendering
