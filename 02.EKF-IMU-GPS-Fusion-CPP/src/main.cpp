// ═══════════════════════════════════════════════════════════════
//  EKF IMU + GPS Sensor Fusion  —  Main
//  Loads fake (or KITTI) data, runs the filter, saves results.
// ═══════════════════════════════════════════════════════════════

#include "ekf_imu_gps.h"
#include "data_loader.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

double rmse(const std::vector<double>& err) {
    double sum = 0.0;
    for (auto e : err) sum += e * e;
    return std::sqrt(sum / err.size());
}

int main(int argc, char** argv) {
    std::string data_dir = "../data/";
    if (argc > 1) data_dir = std::string(argv[1]) + "/";

    // ── Load data ──
    auto imu_data = DataLoader::loadImu(data_dir + "imu.csv");
    auto gps_data = DataLoader::loadGps(data_dir + "gps.csv");
    auto gt_data  = DataLoader::loadGroundTruth(data_dir + "ground_truth.csv");

    if (imu_data.empty() || gps_data.empty()) {
        std::cerr << "No data loaded. Run generate_fake_data first.\n";
        return 1;
    }

    // ── Configure EKF ──
    // Auto-detect sample rate to tune parameters
    double avg_dt = 0.0;
    if (imu_data.size() > 1) {
        avg_dt = (imu_data.back().timestamp - imu_data.front().timestamp)
                 / (imu_data.size() - 1);
    }
    double imu_rate = (avg_dt > 0.0) ? (1.0 / avg_dt) : 100.0;
    bool low_rate = (imu_rate < 50.0);  // KITTI-like low-rate data

    EkfImuGps::Config cfg;
    if (low_rate) {
        // KITTI / low-rate config: trust GPS more, IMU less
        std::cout << "  [Auto] Low-rate IMU detected (" << imu_rate << " Hz) — using KITTI config\n";
        cfg.sigma_acc     = 2.0;
        cfg.sigma_gyro    = 0.05;
        cfg.sigma_ba      = 0.01;
        cfg.sigma_bg      = 0.001;
        cfg.sigma_gps_pos = 0.3;   // KITTI GPS is high precision
        cfg.sigma_gps_vel = 0.05;
    } else {
        // High-rate (fake data) config
        std::cout << "  [Auto] High-rate IMU detected (" << imu_rate << " Hz) — using default config\n";
        cfg.sigma_acc     = 0.5;
        cfg.sigma_gyro    = 0.01;
        cfg.sigma_ba      = 0.001;
        cfg.sigma_bg      = 0.0001;
        cfg.sigma_gps_pos = 1.5;
        cfg.sigma_gps_vel = 0.15;
    }

    EkfImuGps ekf(cfg);

    // Initialize state from first GPS + ground truth (if available)
    {
        StateVec x0 = StateVec::Zero();
        if (!gps_data.empty()) {
            x0(POS_IDX)     = gps_data[0].lat;
            x0(POS_IDX + 1) = gps_data[0].lon;
            x0(POS_IDX + 2) = gps_data[0].alt;
            if (gps_data[0].has_velocity) {
                x0(VEL_IDX)     = gps_data[0].vn;
                x0(VEL_IDX + 1) = gps_data[0].ve;
                x0(VEL_IDX + 2) = gps_data[0].vd;
            }
        }
        if (!gt_data.empty()) {
            x0(ATT_IDX)     = gt_data[0].roll;
            x0(ATT_IDX + 1) = gt_data[0].pitch;
            x0(ATT_IDX + 2) = gt_data[0].yaw;
        }
        ekf.setState(x0);
    }

    // ── Output file for EKF results ──
    std::ofstream ekf_out(data_dir + "ekf_output.csv");
    ekf_out << "timestamp,ekf_x,ekf_y,ekf_z,ekf_vx,ekf_vy,ekf_vz,ekf_roll,ekf_pitch,ekf_yaw\n";

    // Subsample rate: save every N-th sample (all if < 500 samples)
    int save_step = (imu_data.size() > 500) ? 10 : 1;

    // ── Fusion loop ──
    size_t gps_idx = 0;
    double prev_time = imu_data[0].timestamp;

    std::cout << "\n=== Running EKF Fusion ===\n";

    for (size_t i = 0; i < imu_data.size(); ++i) {
        double t = imu_data[i].timestamp;
        double dt = t - prev_time;
        prev_time = t;

        // 1) Predict with IMU
        ekf.predict(imu_data[i], dt);

        // 2) Update with GPS when available
        while (gps_idx < gps_data.size() && gps_data[gps_idx].timestamp <= t + 1e-6) {
            ekf.updateGps(gps_data[gps_idx]);
            gps_idx++;
        }

        // Store output at configured rate
        if (i % save_step == 0) {
            auto pos = ekf.position();
            auto vel = ekf.velocity();
            auto att = ekf.attitude();

            ekf_out << std::fixed << t << ","
                    << pos(0) << "," << pos(1) << "," << pos(2) << ","
                    << vel(0) << "," << vel(1) << "," << vel(2) << ","
                    << att(0) << "," << att(1) << "," << att(2) << "\n";
        }
    }

    ekf_out.close();

    // ── Compute RMSE ──
    std::vector<double> pos_err_x, pos_err_y;
    size_t gt_step = 10;
    size_t n_eval = std::min(imu_data.size() / 10, gt_data.size() / gt_step);

    // Re-run is wasteful, so reload the output
    std::ifstream ekf_in(data_dir + "ekf_output.csv");
    std::string header_line;
    std::getline(ekf_in, header_line);

    size_t gt_i = 0;
    std::string line;
    while (std::getline(ekf_in, line) && gt_i < gt_data.size()) {
        double t, ex, ey, ez, evx, evy, evz, er, ep, eyaw;
        char d;
        std::istringstream ss(line);
        ss >> t >> d >> ex >> d >> ey >> d >> ez >> d
           >> evx >> d >> evy >> d >> evz >> d
           >> er >> d >> ep >> d >> eyaw;

        // Find closest GT
        while (gt_i + 1 < gt_data.size() && gt_data[gt_i].timestamp < t - 0.001) gt_i++;

        if (std::abs(gt_data[gt_i].timestamp - t) < 0.02) {
            pos_err_x.push_back(ex - gt_data[gt_i].px);
            pos_err_y.push_back(ey - gt_data[gt_i].py);
        }
    }

    if (!pos_err_x.empty()) {
        double rmse_x = rmse(pos_err_x);
        double rmse_y = rmse(pos_err_y);
        double rmse_2d = std::sqrt(rmse_x * rmse_x + rmse_y * rmse_y);

        std::cout << "\n=== Results ===\n";
        std::cout << "  Position RMSE X : " << rmse_x  << " m\n";
        std::cout << "  Position RMSE Y : " << rmse_y  << " m\n";
        std::cout << "  Position RMSE 2D: " << rmse_2d << " m\n";
    }

    std::cout << "  GPS measurements used: " << gps_idx << "\n";
    std::cout << "  IMU measurements used: " << imu_data.size() << "\n";
    std::cout << "\n  EKF output saved to: " << data_dir << "ekf_output.csv\n";
    std::cout << "  Run: python3 ../src/plot_results.py to visualize\n";

    return 0;
}
