// ═══════════════════════════════════════════════════════════════
//  Fake IMU + GPS data generator
//  Simulates a vehicle driving a figure-8 trajectory
//  Outputs:  data/imu.csv   data/gps.csv   data/ground_truth.csv
// ═══════════════════════════════════════════════════════════════

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>
#include <string>

struct VehicleState {
    double t;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc_body;   // body-frame accel
    Eigen::Vector3d gyro_body;  // body-frame angular rate
    double roll, pitch, yaw;
};

static Eigen::Matrix3d eulerToR(double r, double p, double y) {
    Eigen::Matrix3d Rz, Ry, Rx;
    Rz << cos(y), -sin(y), 0,  sin(y), cos(y), 0,  0, 0, 1;
    Ry << cos(p), 0, sin(p),   0, 1, 0,  -sin(p), 0, cos(p);
    Rx << 1, 0, 0,   0, cos(r), -sin(r),  0, sin(r), cos(r);
    return Rz * Ry * Rx;
}

int main() {
    // ── Trajectory parameters ──
    const double duration   = 120.0;  // seconds
    const double imu_rate   = 100.0;  // Hz
    const double gps_rate   = 10.0;   // Hz
    const double dt_imu     = 1.0 / imu_rate;
    const double dt_gps     = 1.0 / gps_rate;

    // Figure-8 parameters
    const double A  = 100.0;   // x amplitude [m]
    const double B  = 50.0;    // y amplitude [m]
    const double omega = 2.0 * M_PI / 60.0;  // one loop every 60s

    // Noise parameters
    std::mt19937 rng(42);
    std::normal_distribution<double> acc_noise(0.0, 0.3);    // m/s²
    std::normal_distribution<double> gyro_noise(0.0, 0.005); // rad/s
    std::normal_distribution<double> gps_pos_noise(0.0, 1.5); // m
    std::normal_distribution<double> gps_vel_noise(0.0, 0.1); // m/s

    // Constant biases
    Eigen::Vector3d accel_bias(0.05, -0.03, 0.02);
    Eigen::Vector3d gyro_bias(0.001, -0.0005, 0.0008);

    const Eigen::Vector3d gravity(0.0, 0.0, -9.81);

    // ── Open output files ──
    std::string data_dir = "../data/";
    std::ofstream imu_file(data_dir + "imu.csv");
    std::ofstream gps_file(data_dir + "gps.csv");
    std::ofstream gt_file(data_dir + "ground_truth.csv");

    imu_file << "timestamp,ax,ay,az,gx,gy,gz\n";
    gps_file << "timestamp,lat,lon,alt,vn,ve,vd\n";
    gt_file  << "timestamp,px,py,pz,vx,vy,vz,roll,pitch,yaw\n";

    int imu_count = 0, gps_count = 0;
    double next_gps_time = 0.0;

    for (double t = 0.0; t <= duration; t += dt_imu) {
        // ── True trajectory (figure-8) ──
        double px = A * sin(omega * t);
        double py = B * sin(2.0 * omega * t);
        double pz = 0.0;

        double vx = A * omega * cos(omega * t);
        double vy = B * 2.0 * omega * cos(2.0 * omega * t);
        double vz = 0.0;

        double ax_w = -A * omega * omega * sin(omega * t);
        double ay_w = -B * 4.0 * omega * omega * sin(2.0 * omega * t);
        double az_w = 0.0;

        // Yaw = heading direction
        double yaw   = atan2(vy, vx);
        double roll   = 0.0;
        double pitch  = 0.0;

        // Compute yaw rate (numerical derivative)
        double dt_small = 0.0001;
        double vx2 = A * omega * cos(omega * (t + dt_small));
        double vy2 = B * 2.0 * omega * cos(2.0 * omega * (t + dt_small));
        double yaw2 = atan2(vy2, vx2);
        double dyaw = yaw2 - yaw;
        if (dyaw >  M_PI) dyaw -= 2.0 * M_PI;
        if (dyaw < -M_PI) dyaw += 2.0 * M_PI;
        double yaw_rate = dyaw / dt_small;

        // Body-frame acceleration: R^T * (acc_world - gravity)
        Eigen::Matrix3d R = eulerToR(roll, pitch, yaw);
        Eigen::Vector3d acc_world(ax_w, ay_w, az_w);
        Eigen::Vector3d acc_body = R.transpose() * (acc_world - gravity);

        // Body-frame gyro
        Eigen::Vector3d gyro_body(0.0, 0.0, yaw_rate);

        // ── Write IMU (with noise + bias) ──
        double imu_ax = acc_body(0) + accel_bias(0) + acc_noise(rng);
        double imu_ay = acc_body(1) + accel_bias(1) + acc_noise(rng);
        double imu_az = acc_body(2) + accel_bias(2) + acc_noise(rng);
        double imu_gx = gyro_body(0) + gyro_bias(0) + gyro_noise(rng);
        double imu_gy = gyro_body(1) + gyro_bias(1) + gyro_noise(rng);
        double imu_gz = gyro_body(2) + gyro_bias(2) + gyro_noise(rng);

        imu_file << std::fixed << t << ","
                 << imu_ax << "," << imu_ay << "," << imu_az << ","
                 << imu_gx << "," << imu_gy << "," << imu_gz << "\n";
        imu_count++;

        // ── Write GPS (at lower rate, with noise) ──
        if (t >= next_gps_time - 1e-6) {
            double gps_x = px + gps_pos_noise(rng);
            double gps_y = py + gps_pos_noise(rng);
            double gps_z = pz + gps_pos_noise(rng);
            double gps_vn = vx + gps_vel_noise(rng);
            double gps_ve = vy + gps_vel_noise(rng);
            double gps_vd = vz + gps_vel_noise(rng);

            gps_file << std::fixed << t << ","
                     << gps_x << "," << gps_y << "," << gps_z << ","
                     << gps_vn << "," << gps_ve << "," << gps_vd << "\n";
            gps_count++;
            next_gps_time += dt_gps;
        }

        // ── Write ground truth ──
        gt_file << std::fixed << t << ","
                << px << "," << py << "," << pz << ","
                << vx << "," << vy << "," << vz << ","
                << roll << "," << pitch << "," << yaw << "\n";
    }

    imu_file.close();
    gps_file.close();
    gt_file.close();

    std::cout << "=== Fake Data Generation Complete ===\n";
    std::cout << "  IMU samples : " << imu_count << "\n";
    std::cout << "  GPS samples : " << gps_count << "\n";
    std::cout << "  Duration    : " << duration << " s\n";
    std::cout << "  Trajectory  : figure-8 (" << A << "m x " << B << "m)\n";
    std::cout << "  Files saved to: data/\n";

    return 0;
}
