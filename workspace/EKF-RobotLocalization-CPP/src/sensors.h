#ifndef INCLUDE_EKFRL_SENSORS_H
#define INCLUDE_EKFRL_SENSORS_H

#include <random>
#include <vector>

class LandmarkMap;

struct OdometryMeasurement
{
    double v_left;
    double v_right;
    double v;
    double omega;
};

struct IMUMeasurement
{
    double omega;
};

struct RangeBearingMeasurement
{
    double range;
    double bearing;
    int id;
};

class WheelEncoderSensor
{
    public:

        WheelEncoderSensor();
        void reset();
        void setNoiseStd(double velocity_std);
        void setDriftRate(double drift_per_second);
        void setWheelBase(double wheel_base);
        OdometryMeasurement generateMeasurement(double true_v_left, double true_v_right, double dt);

    private:

        std::mt19937 m_rand_gen;
        double m_velocity_noise_std;
        double m_drift_rate;
        double m_accumulated_drift;
        double m_wheel_base;
};

class IMUSensor
{
    public:

        IMUSensor();
        void reset();
        void setGyroNoiseStd(double std);
        void setGyroBias(double bias);
        IMUMeasurement generateMeasurement(double true_omega);

    private:

        std::mt19937 m_rand_gen;
        double m_noise_std;
        double m_bias;
};

class RangeBearingSensor
{
    public:

        RangeBearingSensor();
        void reset();
        void setNoiseStd(double range_std, double bearing_std);
        void setMaxRange(double range);
        void setFieldOfView(double fov_rad);
        void setDataAssociationEnabled(bool enabled);
        std::vector<RangeBearingMeasurement> generateMeasurements(
            double robot_x, double robot_y, double robot_theta,
            const LandmarkMap& map);

    private:

        std::mt19937 m_rand_gen;
        double m_range_noise_std;
        double m_bearing_noise_std;
        double m_max_range;
        double m_fov;
        bool m_id_enabled;
};

#endif  // INCLUDE_EKFRL_SENSORS_H
