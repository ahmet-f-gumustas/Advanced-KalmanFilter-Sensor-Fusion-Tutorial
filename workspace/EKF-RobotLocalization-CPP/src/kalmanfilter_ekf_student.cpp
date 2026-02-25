// ------------------------------------------------------------------------------- //
// EKF Robot Localization Simulator - Extended Kalman Filter
//
// ####### STUDENT FILE #######
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

        // Implement the EKF Prediction Step for a differential drive robot.
        //
        // HINT: State vector is [px, py, theta] (3x1).
        // HINT: Use odom.v as the linear velocity input from wheel encoders.
        // HINT: Use imu.omega as the angular velocity input from the IMU gyroscope.
        //
        // Steps:
        // 1. Extract current state variables (px, py, theta)
        // 2. Compute the predicted state using the kinematic model:
        //    px_new = px + v * cos(theta) * dt
        //    py_new = py + v * sin(theta) * dt
        //    theta_new = wrapAngle(theta + omega * dt)
        //
        // 3. Compute the Jacobian F (3x3):
        //    F = [1  0  -v*sin(theta)*dt]
        //        [0  1   v*cos(theta)*dt]
        //        [0  0   1              ]
        //
        // 4. Compute process noise covariance Q (3x3):
        //    Use ODOM_VEL_STD and IMU_GYRO_STD constants
        //
        // 5. Update: cov = F * cov * F^T + Q
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE



        // ----------------------------------------------------------------------- //

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

        // Implement the EKF Update Step for range-bearing measurements to landmarks.
        //
        // HINT: State vector is [px, py, theta] (3x1).
        // HINT: Measurement vector is [range, bearing] (2x1).
        //
        // Steps:
        // 1. Look up the landmark position using: map.getLandmarkWithId(meas.id)
        // 2. Compute the predicted measurement z_hat:
        //    delta_x = landmark.x - px
        //    delta_y = landmark.y - py
        //    range_hat = sqrt(delta_x^2 + delta_y^2)
        //    bearing_hat = wrapAngle(atan2(delta_y, delta_x) - theta)
        //
        // 3. Compute the Measurement Jacobian H (2x3):
        //    H = [-delta_x/r   -delta_y/r    0]
        //        [ delta_y/r^2 -delta_x/r^2 -1]
        //
        // 4. Compute innovation: y = z - z_hat, wrap bearing component
        // 5. Compute Kalman Gain: K = P * H^T * (H*P*H^T + R)^-1
        // 6. Update state: x = x + K*y
        // 7. Update covariance: P = (I - K*H) * P
        //
        // HINT: Use wrapAngle() on the bearing innovation.
        // HINT: You can use the constants: RANGE_STD, BEARING_STD
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE

        LandmarkData map_landmark = map.getLandmarkWithId(meas.id);
        if (meas.id != -1 && map_landmark.id != -1)
        {
            // The landmark position: map_landmark.x, map_landmark.y
        }

        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    }
    else
    {
        // Implement filter initialization from the first range-bearing measurement.
        //
        // HINT: You can compute the initial position from the landmark position
        //       and the range/bearing measurement:
        //       px = landmark.x - range * cos(bearing + theta_guess)
        //       py = landmark.y - range * sin(bearing + theta_guess)
        //
        // HINT: You can use the constants: INIT_POS_STD, INIT_THETA_STD
        // ----------------------------------------------------------------------- //
        // ENTER YOUR CODE HERE



        // ----------------------------------------------------------------------- //
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
