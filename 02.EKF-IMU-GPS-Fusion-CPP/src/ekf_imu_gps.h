#pragma once

#include "ekf_types.h"
#include <cmath>
#include <iostream>

// ═══════════════════════════════════════════════
//  Extended Kalman Filter  —  IMU / GPS Fusion
//  15-state: pos(3) vel(3) att(3) ba(3) bg(3)
// ═══════════════════════════════════════════════
class EkfImuGps {
public:
    // ── tuning knobs ──────────────────────────
    struct Config {
        double sigma_acc;
        double sigma_gyro;
        double sigma_ba;
        double sigma_bg;
        double sigma_gps_pos;
        double sigma_gps_vel;
        Eigen::Vector3d gravity;

        Config()
            : sigma_acc(0.5)
            , sigma_gyro(0.01)
            , sigma_ba(0.001)
            , sigma_bg(0.0001)
            , sigma_gps_pos(1.0)
            , sigma_gps_vel(0.1)
            , gravity(0.0, 0.0, -9.81) {}
    };

    explicit EkfImuGps(const Config& cfg = Config()) : cfg_(cfg) {
        x_.setZero();
        P_ = StateMat::Identity() * 1.0;
        // initial bias uncertainty
        P_.block<3,3>(BA_IDX, BA_IDX) = Eigen::Matrix3d::Identity() * 0.01;
        P_.block<3,3>(BG_IDX, BG_IDX) = Eigen::Matrix3d::Identity() * 0.0001;
    }

    // ── Prediction (IMU) ─────────────────────
    void predict(const ImuMeasurement& imu, double dt) {
        if (dt <= 0.0) return;

        // Current attitude
        double roll  = x_(ATT_IDX + 0);
        double pitch = x_(ATT_IDX + 1);
        double yaw   = x_(ATT_IDX + 2);

        Eigen::Matrix3d R = eulerToRotation(roll, pitch, yaw);

        // Bias-corrected IMU
        Eigen::Vector3d acc_body(imu.ax - x_(BA_IDX),
                                 imu.ay - x_(BA_IDX+1),
                                 imu.az - x_(BA_IDX+2));
        Eigen::Vector3d gyro_body(imu.gx - x_(BG_IDX),
                                  imu.gy - x_(BG_IDX+1),
                                  imu.gz - x_(BG_IDX+2));

        // Rotate accel to world frame and remove gravity
        Eigen::Vector3d acc_world = R * acc_body + cfg_.gravity;

        // ── State propagation ──
        // Position
        x_.segment<3>(POS_IDX) += x_.segment<3>(VEL_IDX) * dt
                                 + 0.5 * acc_world * dt * dt;
        // Velocity
        x_.segment<3>(VEL_IDX) += acc_world * dt;

        // Attitude (small-angle integration)
        Eigen::Vector3d dtheta = gyro_body * dt;
        x_(ATT_IDX + 0) += dtheta(0);
        x_(ATT_IDX + 1) += dtheta(1);
        x_(ATT_IDX + 2) += dtheta(2);

        // Normalize yaw to [-pi, pi]
        x_(ATT_IDX + 2) = normalizeAngle(x_(ATT_IDX + 2));

        // ── Jacobian F ──
        StateMat F = StateMat::Identity();

        // dp/dv
        F.block<3,3>(POS_IDX, VEL_IDX) = Eigen::Matrix3d::Identity() * dt;

        // dv/datt  (skew-symmetric of R*acc_body)
        Eigen::Vector3d ra = R * acc_body;
        F.block<3,3>(VEL_IDX, ATT_IDX) = -skew(ra) * dt;

        // dv/dba
        F.block<3,3>(VEL_IDX, BA_IDX) = -R * dt;

        // datt/dbg
        F.block<3,3>(ATT_IDX, BG_IDX) = -Eigen::Matrix3d::Identity() * dt;

        // ── Process noise Q ──
        StateMat Q = StateMat::Zero();
        Q.block<3,3>(VEL_IDX, VEL_IDX) = Eigen::Matrix3d::Identity()
                                          * cfg_.sigma_acc * cfg_.sigma_acc * dt;
        Q.block<3,3>(ATT_IDX, ATT_IDX) = Eigen::Matrix3d::Identity()
                                          * cfg_.sigma_gyro * cfg_.sigma_gyro * dt;
        Q.block<3,3>(BA_IDX, BA_IDX)   = Eigen::Matrix3d::Identity()
                                          * cfg_.sigma_ba * cfg_.sigma_ba * dt;
        Q.block<3,3>(BG_IDX, BG_IDX)   = Eigen::Matrix3d::Identity()
                                          * cfg_.sigma_bg * cfg_.sigma_bg * dt;

        // Covariance propagation
        P_ = F * P_ * F.transpose() + Q;
    }

    // ── GPS Update (position + optional velocity) ──
    void updateGps(const GpsMeasurement& gps) {
        int meas_dim = gps.has_velocity ? 6 : 3;

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(meas_dim, STATE_DIM);
        Eigen::VectorXd z = Eigen::VectorXd::Zero(meas_dim);
        Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(meas_dim);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(meas_dim, meas_dim);

        // Position part
        H.block<3,3>(0, POS_IDX) = Eigen::Matrix3d::Identity();
        z.head<3>() << gps.lat, gps.lon, gps.alt;  // in local frame (ENU metres)
        z_pred.head<3>() = x_.segment<3>(POS_IDX);
        R.block<3,3>(0,0) = Eigen::Matrix3d::Identity()
                             * cfg_.sigma_gps_pos * cfg_.sigma_gps_pos;

        if (gps.has_velocity) {
            H.block<3,3>(3, VEL_IDX) = Eigen::Matrix3d::Identity();
            z.tail<3>() << gps.vn, gps.ve, gps.vd;
            z_pred.tail<3>() = x_.segment<3>(VEL_IDX);
            R.block<3,3>(3,3) = Eigen::Matrix3d::Identity()
                                * cfg_.sigma_gps_vel * cfg_.sigma_gps_vel;
        }

        // Innovation
        Eigen::VectorXd y = z - z_pred;

        // Kalman gain
        Eigen::MatrixXd S = H * P_ * H.transpose() + R;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // State update
        x_ += K * y;
        x_(ATT_IDX + 2) = normalizeAngle(x_(ATT_IDX + 2));

        // Covariance update (Joseph form for numerical stability)
        StateMat I = StateMat::Identity();
        StateMat IKH = I - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
    }

    // ── Accessors ────────────────────────────
    const StateVec&  state()      const { return x_; }
    const StateMat&  covariance() const { return P_; }

    Eigen::Vector3d position() const { return x_.segment<3>(POS_IDX); }
    Eigen::Vector3d velocity() const { return x_.segment<3>(VEL_IDX); }
    Eigen::Vector3d attitude() const { return x_.segment<3>(ATT_IDX); }

    void setState(const StateVec& x) { x_ = x; }
    void setCovariance(const StateMat& P) { P_ = P; }

private:
    Config   cfg_;
    StateVec x_;
    StateMat P_;

    // ── helpers ──────────────────────────────
    static Eigen::Matrix3d eulerToRotation(double r, double p, double y) {
        Eigen::Matrix3d Rz, Ry, Rx;
        Rz << cos(y), -sin(y), 0,
              sin(y),  cos(y), 0,
              0,       0,      1;
        Ry << cos(p),  0, sin(p),
              0,       1, 0,
             -sin(p),  0, cos(p);
        Rx << 1, 0,      0,
              0, cos(r), -sin(r),
              0, sin(r),  cos(r);
        return Rz * Ry * Rx;
    }

    static Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
        Eigen::Matrix3d S;
        S <<  0,    -v(2),  v(1),
              v(2),  0,    -v(0),
             -v(1),  v(0),  0;
        return S;
    }

    static double normalizeAngle(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }
};
