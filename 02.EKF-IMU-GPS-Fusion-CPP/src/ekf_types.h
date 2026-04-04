#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

// ────────────────────────────────────────────
//  IMU measurement  (accelerometer + gyroscope)
// ────────────────────────────────────────────
struct ImuMeasurement {
    double timestamp;       // seconds
    double ax, ay, az;      // accelerometer  [m/s²]
    double gx, gy, gz;      // gyroscope      [rad/s]
};

// ────────────────────────────────────────────
//  GPS measurement  (position + optional velocity)
// ────────────────────────────────────────────
struct GpsMeasurement {
    double timestamp;       // seconds
    double lat, lon, alt;   // [deg, deg, m]  — or local ENU [m, m, m]
    double vn, ve, vd;      // NED velocity   [m/s]  (0 if unavailable)
    bool   has_velocity;
};

// ────────────────────────────────────────────
//  EKF State vector (15-state error-state)
//    [px py pz  vx vy vz  roll pitch yaw  bax bay baz  bgx bgy bgz]
// ────────────────────────────────────────────
constexpr int STATE_DIM = 15;
constexpr int POS_IDX   = 0;   // position  3
constexpr int VEL_IDX   = 3;   // velocity  3
constexpr int ATT_IDX   = 6;   // attitude  3  (euler: roll, pitch, yaw)
constexpr int BA_IDX    = 9;   // accel bias 3
constexpr int BG_IDX    = 12;  // gyro bias  3

using StateVec = Eigen::Matrix<double, STATE_DIM, 1>;
using StateMat = Eigen::Matrix<double, STATE_DIM, STATE_DIM>;

// ────────────────────────────────────────────
//  Ground truth entry (for evaluation)
// ────────────────────────────────────────────
struct GroundTruth {
    double timestamp;
    double px, py, pz;
    double vx, vy, vz;
    double roll, pitch, yaw;
};
