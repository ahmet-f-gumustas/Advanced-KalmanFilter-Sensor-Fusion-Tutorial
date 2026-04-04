# Project 1 — EKF IMU + GPS Sensor Fusion

Extended Kalman Filter (EKF) implementation for fusing **IMU** (accelerometer + gyroscope) and **GPS** (position + velocity) measurements.

## State Vector (15-state)

| Index | State | Description |
|-------|-------|-------------|
| 0-2   | px, py, pz | Position [m] |
| 3-5   | vx, vy, vz | Velocity [m/s] |
| 6-8   | roll, pitch, yaw | Attitude [rad] |
| 9-11  | bax, bay, baz | Accelerometer bias [m/s²] |
| 12-14 | bgx, bgy, bgz | Gyroscope bias [rad/s] |

## Simulated Trajectory

The fake data generator creates a **figure-8** trajectory with:
- IMU at **100 Hz** (with noise + constant bias)
- GPS at **10 Hz** (with Gaussian noise)
- Duration: **120 seconds**

## Build & Run

```bash
# 1. Generate fake data
cd build
cmake .. && make -j$(nproc)
./generate_fake_data

# 2. Run EKF fusion
./ekf_imu_gps_fusion
```

## Dependencies

- **Eigen3** — linear algebra
- **Python3 + NumPy + Matplotlib** — plotting
- **CMake 3.10+**

## Output

- `data/ekf_results.png` — trajectory + position error plots
- `data/ekf_velocity_attitude.png` — velocity and attitude plots
