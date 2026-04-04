#include "simulation.h"
#include "utils.h"

Simulation::Simulation()
: m_sim_parameters(SimulationParams()),
    m_is_paused(false),
    m_is_running(false),
    m_time_multiplier(1),
    m_view_size(10),
    m_time(0.0),
    m_time_till_encoder_measurement(0.0),
    m_time_till_imu_measurement(0.0),
    m_time_till_range_measurement(0.0)
{
    m_last_odom = {0,0,0,0};
    m_last_imu = {0};
}

void Simulation::reset()
{
    m_time = 0.0;
    m_time_till_encoder_measurement = 0.0;
    m_time_till_imu_measurement = 0.0;
    m_time_till_range_measurement = 0.0;

    m_is_running = true;
    m_is_paused = false;

    m_kalman_filter.reset();

    m_encoder_sensor.reset();
    m_encoder_sensor.setNoiseStd(m_sim_parameters.encoder_velocity_noise_std);
    m_encoder_sensor.setDriftRate(m_sim_parameters.encoder_drift_rate);
    m_encoder_sensor.setWheelBase(m_robot.getWheelBase());

    m_imu_sensor.reset();
    m_imu_sensor.setGyroNoiseStd(m_sim_parameters.imu_gyro_noise_std);
    m_imu_sensor.setGyroBias(m_sim_parameters.imu_gyro_bias);

    m_range_sensor.reset();
    m_range_sensor.setNoiseStd(m_sim_parameters.range_noise_std, m_sim_parameters.bearing_noise_std);
    m_range_sensor.setMaxRange(m_sim_parameters.range_sensor_max_range);
    m_range_sensor.setFieldOfView(m_sim_parameters.range_sensor_fov);
    m_range_sensor.setDataAssociationEnabled(m_sim_parameters.range_sensor_id_enabled);

    m_robot.reset(m_sim_parameters.robot_initial_x, m_sim_parameters.robot_initial_y,
                  m_sim_parameters.robot_initial_theta, m_sim_parameters.robot_initial_v);

    for(auto& cmd : m_sim_parameters.robot_commands){m_robot.addRobotCommand(cmd.get());}

    m_range_measurement_history.clear();
    m_robot_position_history.clear();
    m_filter_position_history.clear();

    m_filter_error_x_history.clear();
    m_filter_error_y_history.clear();
    m_filter_error_theta_history.clear();

    m_last_odom = {0,0,0,0};
    m_last_imu = {0};

    std::cout << "Simulation: Reset" << std::endl;
}

void Simulation::update()
{
    if (m_is_running && !m_is_paused)
    {
        for (unsigned i = 0; i < (unsigned)m_time_multiplier; ++i)
        {
            if(m_time >= m_sim_parameters.end_time)
            {
                m_is_running = false;
                std::cout << "Simulation: Reached End of Simulation Time (" << m_time << ")" << std::endl;
                return;
            }

            // Update Robot Motion
            m_robot.update(m_time, m_sim_parameters.time_step);
            m_robot_position_history.push_back(Vector2(m_robot.getRobotState().x, m_robot.getRobotState().y));

            // Wheel Encoder + IMU Measurement -> Prediction Step
            // Encoder and IMU are combined for the prediction step
            if (m_sim_parameters.encoder_enabled)
            {
                if (m_time_till_encoder_measurement <= 0)
                {
                    m_last_odom = m_encoder_sensor.generateMeasurement(
                        m_robot.getLeftWheelSpeed(), m_robot.getRightWheelSpeed(),
                        m_sim_parameters.time_step);
                    m_time_till_encoder_measurement += 1.0/m_sim_parameters.encoder_update_rate;
                }
                m_time_till_encoder_measurement -= m_sim_parameters.time_step;
            }

            if (m_sim_parameters.imu_enabled)
            {
                if (m_time_till_imu_measurement <= 0)
                {
                    m_last_imu = m_imu_sensor.generateMeasurement(m_robot.getRobotState().omega);

                    // Run prediction step with combined odometry + IMU
                    m_kalman_filter.predictionStep(m_last_odom, m_last_imu, m_sim_parameters.time_step);
                    m_time_till_imu_measurement += 1.0/m_sim_parameters.imu_update_rate;
                }
                m_time_till_imu_measurement -= m_sim_parameters.time_step;
            }

            // Range-Bearing Measurement -> Update Step
            if (m_sim_parameters.range_sensor_enabled)
            {
                if (m_time_till_range_measurement <= 0)
                {
                    std::vector<RangeBearingMeasurement> range_measurements = m_range_sensor.generateMeasurements(
                        m_robot.getRobotState().x, m_robot.getRobotState().y,
                        m_robot.getRobotState().theta, m_landmarks);
                    m_kalman_filter.handleRangeBearingMeasurements(range_measurements, m_landmarks);
                    m_range_measurement_history = range_measurements;
                    m_time_till_range_measurement += 1.0/m_sim_parameters.range_sensor_update_rate;
                }
                m_time_till_range_measurement -= m_sim_parameters.time_step;
            }

            // Save Filter History and Calculate Error Stats
            if (m_kalman_filter.isInitialised())
            {
                RobotState robot_state = m_robot.getRobotState();
                RobotState filter_state = m_kalman_filter.getRobotState();
                m_filter_position_history.push_back(Vector2(filter_state.x, filter_state.y));
                m_filter_error_x_history.push_back(filter_state.x - robot_state.x);
                m_filter_error_y_history.push_back(filter_state.y - robot_state.y);
                m_filter_error_theta_history.push_back(wrapAngle(filter_state.theta - robot_state.theta));
            }

            m_time += m_sim_parameters.time_step;
        }
    }
}

void Simulation::render(Display& disp)
{
    // Set view centered on robot
    disp.setView(m_view_size * disp.getScreenAspectRatio(), m_view_size,
                 m_robot.getRobotState().x, m_robot.getRobotState().y);

    // Render environment
    m_landmarks.render(disp);

    // Render robot
    m_robot.render(disp);

    // Ground truth path (dark green)
    disp.setDrawColour(0, 100, 0);
    disp.drawLines(m_robot_position_history);

    // Filter estimated path (dark red)
    disp.setDrawColour(100, 0, 0);
    disp.drawLines(m_filter_position_history);

    // Filter state marker and covariance ellipse
    if (m_kalman_filter.isInitialised())
    {
        RobotState filter_state = m_kalman_filter.getRobotState();
        Eigen::Matrix2d cov = m_kalman_filter.getRobotStatePositionCovariance();

        double x = filter_state.x;
        double y = filter_state.y;
        double sigma_xx = cov(0,0);
        double sigma_yy = cov(1,1);
        double sigma_xy = cov(0,1);

        // X marker at filter position
        double ms = 0.15;
        std::vector<Vector2> marker1 = {{x+ms,y+ms},{x-ms,y-ms}};
        std::vector<Vector2> marker2 = {{x+ms,y-ms},{x-ms,y+ms}};
        disp.setDrawColour(255, 0, 0);
        disp.drawLines(marker1);
        disp.drawLines(marker2);

        // Covariance ellipse
        std::vector<Vector2> cov_world = generateEllipse(x, y, sigma_xx, sigma_yy, sigma_xy);
        disp.setDrawColour(255, 0, 0);
        disp.drawLines(cov_world);
    }

    // Render Range-Bearing Measurements (yellow lines from robot to measured landmark position)
    for(const auto& meas : m_range_measurement_history)
    {
        double x0 = m_robot.getRobotState().x;
        double y0 = m_robot.getRobotState().y;
        double delta_x = meas.range * cos(meas.bearing + m_robot.getRobotState().theta);
        double delta_y = meas.range * sin(meas.bearing + m_robot.getRobotState().theta);
        disp.setDrawColour(201, 201, 0);
        disp.drawLine(Vector2(x0, y0), Vector2(x0 + delta_x, y0 + delta_y));
    }

    // HUD Text
    int x_offset, y_offset;
    int stride = 20;

    // Simulation Status (top-left)
    x_offset = 10;
    y_offset = 30;
    std::string profile_string = string_format("Profile: %s", m_sim_parameters.profile_name.c_str());
    std::string time_string = string_format("Time: %0.2f (x%d)", m_time, m_time_multiplier);
    std::string encoder_string = string_format("Encoder: %s (%0.1f Hz)", (m_sim_parameters.encoder_enabled ? "ON" : "OFF"), m_sim_parameters.encoder_update_rate);
    std::string imu_string = string_format("IMU: %s (%0.1f Hz)", (m_sim_parameters.imu_enabled ? "ON" : "OFF"), m_sim_parameters.imu_update_rate);
    std::string range_string = string_format("Range Sensor: %s (%0.1f Hz)", (m_sim_parameters.range_sensor_enabled ? "ON" : "OFF"), m_sim_parameters.range_sensor_update_rate);
    disp.drawText_MainFont(profile_string, Vector2(x_offset, y_offset+stride*-1), 1.0, {255,255,255});
    disp.drawText_MainFont(time_string, Vector2(x_offset, y_offset+stride*0), 1.0, {255,255,255});
    disp.drawText_MainFont(encoder_string, Vector2(x_offset, y_offset+stride*1), 1.0, {255,255,255});
    disp.drawText_MainFont(imu_string, Vector2(x_offset, y_offset+stride*2), 1.0, {255,255,255});
    disp.drawText_MainFont(range_string, Vector2(x_offset, y_offset+stride*3), 1.0, {255,255,255});
    if (m_is_paused){disp.drawText_MainFont("PAUSED", Vector2(x_offset, y_offset+stride*4), 1.0, {255,0,0});}
    if (!m_is_running){disp.drawText_MainFont("FINISHED", Vector2(x_offset, y_offset+stride*5), 1.0, {255,0,0});}

    // Robot State (top-right)
    x_offset = 800;
    y_offset = 10;
    std::string xpos = string_format("X Position: %0.3f m", m_robot.getRobotState().x);
    std::string ypos = string_format("Y Position: %0.3f m", m_robot.getRobotState().y);
    std::string heading = string_format("   Heading: %0.2f deg", m_robot.getRobotState().theta * 180.0/M_PI);
    std::string velocity = string_format("    Velocity: %0.3f m/s", m_robot.getRobotState().v);
    disp.drawText_MainFont("Robot State", Vector2(x_offset-5, y_offset+stride*0), 1.0, {255,255,255});
    disp.drawText_MainFont(xpos, Vector2(x_offset, y_offset+stride*1), 1.0, {255,255,255});
    disp.drawText_MainFont(ypos, Vector2(x_offset, y_offset+stride*2), 1.0, {255,255,255});
    disp.drawText_MainFont(heading, Vector2(x_offset, y_offset+stride*3), 1.0, {255,255,255});
    disp.drawText_MainFont(velocity, Vector2(x_offset, y_offset+stride*4), 1.0, {255,255,255});

    // Filter State
    std::string kf_xpos = string_format("X Position: %0.3f m", m_kalman_filter.getRobotState().x);
    std::string kf_ypos = string_format("Y Position: %0.3f m", m_kalman_filter.getRobotState().y);
    std::string kf_heading = string_format("   Heading: %0.2f deg", m_kalman_filter.getRobotState().theta * 180.0/M_PI);
    disp.drawText_MainFont("Filter State", Vector2(x_offset, y_offset+stride*6), 1.0, {255,255,255});
    disp.drawText_MainFont(kf_xpos, Vector2(x_offset, y_offset+stride*7), 1.0, {255,255,255});
    disp.drawText_MainFont(kf_ypos, Vector2(x_offset, y_offset+stride*8), 1.0, {255,255,255});
    disp.drawText_MainFont(kf_heading, Vector2(x_offset, y_offset+stride*9), 1.0, {255,255,255});

    // Keyboard Controls (bottom-left)
    x_offset = 10;
    y_offset = 650;
    disp.drawText_MainFont("Reset Key: r", Vector2(x_offset, y_offset+stride*0), 1.0, {255,255,255});
    disp.drawText_MainFont("Pause Key: [space bar]", Vector2(x_offset, y_offset+stride*1), 1.0, {255,255,255});
    disp.drawText_MainFont("Speed Multiplier (+/-) Key: [ / ]", Vector2(x_offset, y_offset+stride*2), 1.0, {255,255,255});
    disp.drawText_MainFont("Zoom (+/-) Key: + / - (keypad)", Vector2(x_offset, y_offset+stride*3), 1.0, {255,255,255});
    disp.drawText_MainFont("Motion Profile Key: 1 - 8", Vector2(x_offset, y_offset+stride*4), 1.0, {255,255,255});

    // Error Metrics (bottom-right)
    x_offset = 750;
    y_offset = 650;
    std::string xpos_error = string_format("X Position RMSE: %0.3f m", calculateRMSE(m_filter_error_x_history));
    std::string ypos_error = string_format("Y Position RMSE: %0.3f m", calculateRMSE(m_filter_error_y_history));
    std::string heading_error = string_format("   Heading RMSE: %0.2f deg", 180.0/M_PI * calculateRMSE(m_filter_error_theta_history));
    disp.drawText_MainFont(xpos_error, Vector2(x_offset, y_offset+stride*0), 1.0, {255,255,255});
    disp.drawText_MainFont(ypos_error, Vector2(x_offset, y_offset+stride*1), 1.0, {255,255,255});
    disp.drawText_MainFont(heading_error, Vector2(x_offset, y_offset+stride*2), 1.0, {255,255,255});
}

void Simulation::reset(SimulationParams sim_params){m_sim_parameters = sim_params; reset();}

void Simulation::increaseTimeMultiplier()
{
    m_time_multiplier++;
    std::cout << "Simulation: Time Multiplier Increased (x" << m_time_multiplier << ")" << std::endl;
}
void Simulation::decreaseTimeMultiplier()
{
    if (m_time_multiplier > 1)
    {
        m_time_multiplier--;
        std::cout << "Simulation: Time Multiplier Decreased (x" << m_time_multiplier << ")" << std::endl;
    }
}
void Simulation::setTimeMultiplier(unsigned int multiplier){m_time_multiplier = static_cast<int>(multiplier);}

void Simulation::increaseZoom()
{
    if (m_view_size > 2){m_view_size -= 2;}
    std::cout << "Simulation: Zoom Increased (" << m_view_size << "m)" << std::endl;
}
void Simulation::decreaseZoom()
{
    if (m_view_size < 40){m_view_size += 2;}
    std::cout << "Simulation: Zoom Decreased (" << m_view_size << "m)" << std::endl;
}

void Simulation::togglePauseSimulation()
{
    m_is_paused = !m_is_paused;
    std::cout << "Simulation: Paused (" << (m_is_paused?"True":"False") << ")" << std::endl;
}
bool Simulation::isPaused(){return m_is_paused;}
bool Simulation::isRunning(){return m_is_running;}
