#ifndef INCLUDE_EKFRL_SIMULATION_H
#define INCLUDE_EKFRL_SIMULATION_H

#include <memory>
#include <vector>

#include "kalmanfilter.h"
#include "display.h"
#include "robot.h"
#include "landmarks.h"
#include "sensors.h"

struct SimulationParams
{
    std::string profile_name;
    double time_step;
    double end_time;

    bool encoder_enabled;
    double encoder_update_rate;
    double encoder_velocity_noise_std;
    double encoder_drift_rate;

    bool imu_enabled;
    double imu_update_rate;
    double imu_gyro_noise_std;
    double imu_gyro_bias;

    bool range_sensor_enabled;
    bool range_sensor_id_enabled;
    double range_sensor_update_rate;
    double range_noise_std;
    double bearing_noise_std;
    double range_sensor_max_range;
    double range_sensor_fov;

    double robot_initial_x;
    double robot_initial_y;
    double robot_initial_theta;
    double robot_initial_v;

    std::vector<std::shared_ptr<MotionCommandBase>> robot_commands;

    SimulationParams()
    :profile_name(""),
     time_step(0.05), end_time(60),
     encoder_enabled(true), encoder_update_rate(10.0), encoder_velocity_noise_std(0.05), encoder_drift_rate(0.0),
     imu_enabled(true), imu_update_rate(10.0), imu_gyro_noise_std(0.005), imu_gyro_bias(0.0),
     range_sensor_enabled(true), range_sensor_id_enabled(true), range_sensor_update_rate(5.0),
     range_noise_std(0.5), bearing_noise_std(0.02), range_sensor_max_range(10.0), range_sensor_fov(2.0*M_PI),
     robot_initial_x(2.0), robot_initial_y(2.0), robot_initial_theta(0.0), robot_initial_v(0.0)
    {}
};

class Simulation
{
    public:

        Simulation();
        void reset();
        void reset(SimulationParams sim_params);
        void update();
        void render(Display& disp);
        void increaseTimeMultiplier();
        void decreaseTimeMultiplier();
        void setTimeMultiplier(unsigned int multiplier);
        void increaseZoom();
        void decreaseZoom();
        void togglePauseSimulation();
        bool isPaused();
        bool isRunning();

    private:

        SimulationParams m_sim_parameters;
        KalmanFilter m_kalman_filter;
        Robot m_robot;
        LandmarkMap m_landmarks;
        WheelEncoderSensor m_encoder_sensor;
        IMUSensor m_imu_sensor;
        RangeBearingSensor m_range_sensor;

        bool m_is_paused;
        bool m_is_running;
        int  m_time_multiplier;
        double m_view_size;

        double m_time;
        double m_time_till_encoder_measurement;
        double m_time_till_imu_measurement;
        double m_time_till_range_measurement;

        std::vector<RangeBearingMeasurement> m_range_measurement_history;

        std::vector<Vector2> m_robot_position_history;
        std::vector<Vector2> m_filter_position_history;

        std::vector<double> m_filter_error_x_history;
        std::vector<double> m_filter_error_y_history;
        std::vector<double> m_filter_error_theta_history;

        OdometryMeasurement m_last_odom;
        IMUMeasurement m_last_imu;
};

#endif  // INCLUDE_EKFRL_SIMULATION_H
