#include "sensors.h"
#include "landmarks.h"
#include "utils.h"

// Wheel Encoder Sensor
WheelEncoderSensor::WheelEncoderSensor()
    :m_rand_gen(std::mt19937()),m_velocity_noise_std(0.05),m_drift_rate(0.0),m_accumulated_drift(0.0),m_wheel_base(0.3)
{}

void WheelEncoderSensor::reset()
{
    m_rand_gen = std::mt19937();
    m_accumulated_drift = 0.0;
}

void WheelEncoderSensor::setNoiseStd(double velocity_std){m_velocity_noise_std = velocity_std;}
void WheelEncoderSensor::setDriftRate(double drift_per_second){m_drift_rate = drift_per_second;}
void WheelEncoderSensor::setWheelBase(double wheel_base){m_wheel_base = wheel_base;}

OdometryMeasurement WheelEncoderSensor::generateMeasurement(double true_v_left, double true_v_right, double dt)
{
    OdometryMeasurement meas;
    std::normal_distribution<double> noise_dis(0.0, m_velocity_noise_std);

    m_accumulated_drift += m_drift_rate * dt;

    meas.v_left = true_v_left + noise_dis(m_rand_gen) + m_accumulated_drift;
    meas.v_right = true_v_right + noise_dis(m_rand_gen) + m_accumulated_drift;
    meas.v = (meas.v_right + meas.v_left) / 2.0;
    meas.omega = (meas.v_right - meas.v_left) / m_wheel_base;

    return meas;
}

// IMU Sensor
IMUSensor::IMUSensor():m_rand_gen(std::mt19937()),m_noise_std(0.005),m_bias(0.0){}
void IMUSensor::reset(){m_rand_gen = std::mt19937();}
void IMUSensor::setGyroNoiseStd(double std){m_noise_std = std;}
void IMUSensor::setGyroBias(double bias){m_bias = bias;}

IMUMeasurement IMUSensor::generateMeasurement(double true_omega)
{
    IMUMeasurement meas;
    std::normal_distribution<double> gyro_dis(0.0, m_noise_std);
    meas.omega = true_omega + m_bias + gyro_dis(m_rand_gen);
    return meas;
}

// Range-Bearing Sensor
RangeBearingSensor::RangeBearingSensor()
    :m_rand_gen(std::mt19937()),m_range_noise_std(0.5),m_bearing_noise_std(0.02),m_max_range(10.0),m_fov(2.0*M_PI),m_id_enabled(true)
{}

void RangeBearingSensor::reset(){m_rand_gen = std::mt19937();}
void RangeBearingSensor::setNoiseStd(double range_std, double bearing_std){m_range_noise_std = range_std; m_bearing_noise_std = bearing_std;}
void RangeBearingSensor::setMaxRange(double range){m_max_range = range;}
void RangeBearingSensor::setFieldOfView(double fov_rad){m_fov = fov_rad;}
void RangeBearingSensor::setDataAssociationEnabled(bool enabled){m_id_enabled = enabled;}

std::vector<RangeBearingMeasurement> RangeBearingSensor::generateMeasurements(
    double robot_x, double robot_y, double robot_theta, const LandmarkMap& map)
{
    std::vector<RangeBearingMeasurement> meas;
    std::normal_distribution<double> range_dis(0.0, m_range_noise_std);
    std::normal_distribution<double> bearing_dis(0.0, m_bearing_noise_std);

    for (const auto& lm : map.getLandmarks())
    {
        double delta_x = lm.x - robot_x;
        double delta_y = lm.y - robot_y;
        double true_range = std::sqrt(delta_x*delta_x + delta_y*delta_y);
        double true_bearing = wrapAngle(atan2(delta_y, delta_x) - robot_theta);

        if (true_range < m_max_range && std::fabs(true_bearing) <= m_fov / 2.0)
        {
            RangeBearingMeasurement lm_meas;
            lm_meas.range = std::abs(true_range + range_dis(m_rand_gen));
            lm_meas.bearing = wrapAngle(true_bearing + bearing_dis(m_rand_gen));
            lm_meas.id = (m_id_enabled ? lm.id : -1);
            meas.push_back(lm_meas);
        }
    }
    return meas;
}
