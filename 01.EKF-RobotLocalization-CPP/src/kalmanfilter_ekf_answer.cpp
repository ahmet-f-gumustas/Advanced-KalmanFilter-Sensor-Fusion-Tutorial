// ------------------------------------------------------------------------------- //
// EKF Robot Localization Simulator - Extended Kalman Filter
//
// ####### ANSWER FILE #######
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
constexpr double ODOM_VEL_STD = 0.1;
constexpr double IMU_GYRO_STD = 0.01;
constexpr double INIT_POS_STD = 0.5;
constexpr double INIT_THETA_STD = 15.0/180.0 * M_PI;
constexpr double RANGE_STD = 0.5;
constexpr double BEARING_STD = 0.02;
// -------------------------------------------------- //

void KalmanFilter::predictionStep(OdometryMeasurement odom, IMUMeasurement imu, double dt)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Extract current state
        double px = state(0);
        double py = state(1);
        double theta = state(2);

        // Control inputs from sensors
        double v = odom.v;
        double omega = imu.omega;

        // State prediction (nonlinear process model)
        double px_new = px + v * cos(theta) * dt;
        double py_new = py + v * sin(theta) * dt;
        double theta_new = wrapAngle(theta + omega * dt);
        state << px_new, py_new, theta_new;

        // Jacobian F = df/dx (3x3)
        Matrix3d F = Matrix3d::Identity();
        F(0,2) = -v * sin(theta) * dt;
        F(1,2) =  v * cos(theta) * dt;

        // Process noise covariance Q (3x3)
        Matrix3d Q = Matrix3d::Zero();
        Q(0,0) = ODOM_VEL_STD * dt * ODOM_VEL_STD * dt;
        Q(1,1) = ODOM_VEL_STD * dt * ODOM_VEL_STD * dt;
        Q(2,2) = IMU_GYRO_STD * dt * IMU_GYRO_STD * dt;

        // Covariance prediction
        cov = F * cov * F.transpose() + Q;

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilter::handleRangeBearingMeasurements(const std::vector<RangeBearingMeasurement>& dataset, const LandmarkMap& map)
{
    // Assume No Correlation between the Measurements and Update Sequentially
    for(const auto& meas : dataset) {handleRangeBearingMeasurement(meas, map);}
}

void KalmanFilter::handleRangeBearingMeasurement(RangeBearingMeasurement meas, const LandmarkMap& map)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        LandmarkData map_landmark = map.getLandmarkWithId(meas.id);
        if (meas.id != -1 && map_landmark.id != -1)
        {
            double px = state(0);
            double py = state(1);
            double theta = state(2);

            // Measurement vector
            VectorXd z = Vector2d::Zero();
            z << meas.range, meas.bearing;

            // Predicted measurement (nonlinear measurement model)
            double delta_x = map_landmark.x - px;
            double delta_y = map_landmark.y - py;
            double zhat_range = sqrt(delta_x*delta_x + delta_y*delta_y);
            double zhat_bearing = wrapAngle(atan2(delta_y, delta_x) - theta);

            VectorXd z_hat = Vector2d::Zero();
            z_hat << zhat_range, zhat_bearing;

            // Measurement Jacobian H = dh/dx (2x3)
            MatrixXd H = MatrixXd(2, 3);
            H << -delta_x/zhat_range, -delta_y/zhat_range,  0,
                  delta_y/(zhat_range*zhat_range), -delta_x/(zhat_range*zhat_range), -1;

            // Measurement noise covariance R (2x2)
            Matrix2d R = Matrix2d::Zero();
            R(0,0) = RANGE_STD * RANGE_STD;
            R(1,1) = BEARING_STD * BEARING_STD;

            // Innovation
            VectorXd y = z - z_hat;
            y(1) = wrapAngle(y(1)); // Wrap bearing innovation

            // Kalman gain
            MatrixXd S = H * cov * H.transpose() + R;
            MatrixXd K = cov * H.transpose() * S.inverse();

            // State and covariance update
            state = state + K * y;
            state(2) = wrapAngle(state(2));
            cov = (Matrix3d::Identity() - K * H) * cov;
        }

        setState(state);
        setCovariance(cov);
    }
    else
    {
        // Initialize filter from first range-bearing measurement
        LandmarkData map_landmark = map.getLandmarkWithId(meas.id);
        if (meas.id != -1 && map_landmark.id != -1)
        {
            // Estimate initial position from landmark position and measurement
            double init_theta = 0.0; // Assume initial heading unknown
            double init_px = map_landmark.x - meas.range * cos(meas.bearing + init_theta);
            double init_py = map_landmark.y - meas.range * sin(meas.bearing + init_theta);

            VectorXd state = Vector3d::Zero();
            state << init_px, init_py, init_theta;

            Matrix3d cov = Matrix3d::Zero();
            cov(0,0) = INIT_POS_STD * INIT_POS_STD;
            cov(1,1) = INIT_POS_STD * INIT_POS_STD;
            cov(2,2) = INIT_THETA_STD * INIT_THETA_STD;

            setState(state);
            setCovariance(cov);
        }
    }
}

RobotState KalmanFilter::getRobotState()
{
    if (isInitialised())
    {
        VectorXd state = getState(); // STATE VECTOR [px, py, theta]
        return RobotState(state[0], state[1], state[2]);
    }
    return RobotState();
}

Matrix2d KalmanFilter::getRobotStatePositionCovariance()
{
    Matrix2d pos_cov = Matrix2d::Zero();
    MatrixXd cov = getCovariance();
    if (isInitialised() && cov.size() != 0){pos_cov << cov(0,0), cov(0,1), cov(1,0), cov(1,1);}
    return pos_cov;
}
