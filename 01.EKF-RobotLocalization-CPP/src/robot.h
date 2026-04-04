#ifndef INCLUDE_EKFRL_ROBOT_H
#define INCLUDE_EKFRL_ROBOT_H

#include <queue>
#include <cmath>
#include "display.h"
#include "utils.h"

struct RobotState
{
    double x, y, theta;
    double v, omega;
    RobotState():x(0.0), y(0.0), theta(0.0), v(0.0), omega(0.0) {}
    RobotState(double _x, double _y, double _theta):x(_x), y(_y), theta(_theta), v(0.0), omega(0.0) {}
    RobotState(double _x, double _y, double _theta, double _v, double _omega):x(_x), y(_y), theta(_theta), v(_v), omega(_omega) {}
};

class MotionCommandBase
{
    public:
        MotionCommandBase():m_velocity_command(0.0),m_omega_command(0.0),m_start_time(0.0){}
        virtual ~MotionCommandBase(){}
        virtual void startCommand(double time, RobotState state){m_start_time = time; m_start_state = state;}
        virtual bool update(double time, double dt, RobotState state){return false;}
        virtual double getVelocityCommand(){return m_velocity_command;}
        virtual double getOmegaCommand(){return m_omega_command;}
    protected:
        double m_velocity_command, m_omega_command, m_start_time;
        RobotState m_start_state;
};

class MotionCommandStraight : public MotionCommandBase
{
    public:
    MotionCommandStraight(double command_time, double command_velocity)
        :m_command_time(command_time),m_command_velocity(command_velocity){}
    bool update(double time, double dt, RobotState state)
    {
        m_velocity_command = m_command_velocity;
        m_omega_command = 0.0;
        return time > (m_start_time + m_command_time);
    }
    private:
        double m_command_time, m_command_velocity;
};

class MotionCommandTurnInPlace : public MotionCommandBase
{
    public:
    MotionCommandTurnInPlace(double target_heading, double command_omega)
        :m_target_heading(target_heading),m_command_omega(command_omega){}
    bool update(double time, double dt, RobotState state)
    {
        m_velocity_command = 0.0;
        double angle_error = wrapAngle(m_target_heading - state.theta);
        m_omega_command = (angle_error > 0 ? 1.0 : -1.0) * std::fabs(m_command_omega);
        return std::fabs(angle_error) < 0.01;
    }
    private:
        double m_target_heading, m_command_omega;
};

class MotionCommandMoveTo : public MotionCommandBase
{
    public:
    MotionCommandMoveTo(double command_x, double command_y, double command_velocity)
        :m_command_x(command_x),m_command_y(command_y),m_command_velocity(command_velocity){}
    bool update(double time, double dt, RobotState state)
    {
        m_velocity_command = m_command_velocity;
        double delta_x = m_command_x - state.x;
        double delta_y = m_command_y - state.y;
        double range = sqrt(delta_x*delta_x + delta_y*delta_y);
        double angle_command = atan2(delta_y, delta_x);
        double angle_error = wrapAngle(angle_command - state.theta);
        m_omega_command = 2.0 * angle_error;
        return (range < 0.3);
    }
    private:
        double m_command_x, m_command_y, m_command_velocity;
};

class DifferentialDriveMotion
{
    public:

        DifferentialDriveMotion()
            :m_initial_state(RobotState()),m_wheel_base(0.3),m_max_velocity(2.0),m_max_acceleration(1.0),m_max_omega(3.0)
        {reset();}

        DifferentialDriveMotion(double x0, double y0, double theta0, double v0)
            :m_initial_state(RobotState(x0,y0,theta0,v0,0.0)),m_wheel_base(0.3),m_max_velocity(2.0),m_max_acceleration(1.0),m_max_omega(3.0)
        {reset();}

        void reset()
        {
            m_current_state = m_initial_state;
            m_velocity_command = m_initial_state.v;
            m_omega_command = m_initial_state.omega;
        }
        void reset(RobotState state)
        {
            m_initial_state = state;
            reset();
        }

        void update(double dt)
        {
            double cosTheta = cos(m_current_state.theta);
            double sinTheta = sin(m_current_state.theta);

            double accel = m_velocity_command - m_current_state.v;
            if (accel > m_max_acceleration * dt) {accel = m_max_acceleration * dt;}
            if (accel < -m_max_acceleration * dt) {accel = -m_max_acceleration * dt;}
            double vel = m_current_state.v + accel;
            if (vel > m_max_velocity) {vel = m_max_velocity;}
            if (vel < -m_max_velocity) {vel = -m_max_velocity;}

            double omega = m_omega_command;
            if (omega > m_max_omega) {omega = m_max_omega;}
            if (omega < -m_max_omega) {omega = -m_max_omega;}

            double x = m_current_state.x + vel * cosTheta * dt;
            double y = m_current_state.y + vel * sinTheta * dt;
            double theta = wrapAngle(m_current_state.theta + omega * dt);

            // Compute individual wheel speeds for encoder simulation
            m_v_left = vel - (omega * m_wheel_base / 2.0);
            m_v_right = vel + (omega * m_wheel_base / 2.0);

            m_current_state = RobotState(x, y, theta, vel, omega);
        }

        void setVelocityCmd(double vel){m_velocity_command = vel;}
        void setOmegaCmd(double omega){m_omega_command = omega;}
        RobotState getRobotState() const {return m_current_state;}
        double getLeftWheelSpeed() const {return m_v_left;}
        double getRightWheelSpeed() const {return m_v_right;}
        double getWheelBase() const {return m_wheel_base;}

    private:

        RobotState m_current_state;
        RobotState m_initial_state;

        double m_velocity_command;
        double m_omega_command;

        double m_wheel_base;
        double m_max_velocity;
        double m_max_acceleration;
        double m_max_omega;

        double m_v_left = 0.0;
        double m_v_right = 0.0;
};

class Robot
{
    public:

        Robot():m_motion_model(),m_current_command(nullptr)
        {
            // Robot body: circle approximated by line segments (radius ~0.15m)
            int num_pts = 20;
            for (int i = 0; i <= num_pts; ++i)
            {
                double angle = 2.0 * M_PI * i / num_pts;
                m_body_circle.push_back(Vector2(0.15 * cos(angle), 0.15 * sin(angle)));
            }

            // Heading indicator line from center to front
            m_heading_line = {{0, 0}, {0.22, 0}};

            // Left wheel (small rectangle on left side)
            m_left_wheel = {{-0.04, -0.18}, {0.04, -0.18}, {0.04, -0.15}, {-0.04, -0.15}, {-0.04, -0.18}};

            // Right wheel (small rectangle on right side)
            m_right_wheel = {{-0.04, 0.15}, {0.04, 0.15}, {0.04, 0.18}, {-0.04, 0.18}, {-0.04, 0.15}};
        }

        void reset(double x0, double y0, double theta0, double v0)
        {
            m_motion_model.reset(RobotState(x0, y0, theta0, v0, 0.0));
            while (!m_robot_commands.empty()){m_robot_commands.pop();}
            m_current_command = nullptr;
        }

        void addRobotCommand(MotionCommandBase* cmd)
        {
            if (cmd != nullptr){m_robot_commands.push(cmd);}
        }

        RobotState getRobotState() const {return m_motion_model.getRobotState();}
        double getLeftWheelSpeed() const {return m_motion_model.getLeftWheelSpeed();}
        double getRightWheelSpeed() const {return m_motion_model.getRightWheelSpeed();}
        double getWheelBase() const {return m_motion_model.getWheelBase();}

        bool update(double time, double dt)
        {
            if(m_current_command == nullptr && !m_robot_commands.empty())
            {
                m_current_command = m_robot_commands.front();
                m_robot_commands.pop();
                m_current_command->startCommand(time, m_motion_model.getRobotState());
            }

            if (m_current_command != nullptr)
            {
                bool cmd_complete = m_current_command->update(time, dt, m_motion_model.getRobotState());
                m_motion_model.setVelocityCmd(m_current_command->getVelocityCommand());
                m_motion_model.setOmegaCmd(m_current_command->getOmegaCommand());
                if(cmd_complete){m_current_command = nullptr;}
            }
            else
            {
                m_motion_model.setVelocityCmd(0.0);
                m_motion_model.setOmegaCmd(0.0);
            }

            m_motion_model.update(dt);
            return true;
        }

        void render(Display& disp)
        {
            double robotTheta = m_motion_model.getRobotState().theta;
            Vector2 robotPos = Vector2(m_motion_model.getRobotState().x, m_motion_model.getRobotState().y);

            disp.setDrawColour(0, 255, 0);
            disp.drawLines(transformPoints(m_body_circle, robotPos, robotTheta));

            // Heading indicator
            std::vector<Vector2> heading_world = transformPoints(m_heading_line, robotPos, robotTheta);
            disp.drawLines(heading_world);

            // Wheels
            disp.setDrawColour(0, 201, 0);
            disp.drawLines(transformPoints(m_left_wheel, robotPos, robotTheta));
            disp.drawLines(transformPoints(m_right_wheel, robotPos, robotTheta));
        }

    private:

        DifferentialDriveMotion m_motion_model;
        MotionCommandBase* m_current_command;
        std::queue<MotionCommandBase*> m_robot_commands;

        std::vector<Vector2> m_body_circle;
        std::vector<Vector2> m_heading_line;
        std::vector<Vector2> m_left_wheel;
        std::vector<Vector2> m_right_wheel;
};

#endif  // INCLUDE_EKFRL_ROBOT_H
