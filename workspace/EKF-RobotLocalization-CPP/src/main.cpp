#include <string>
#include <iostream>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

#include "simulation.h"
#include "robot.h"
#include "display.h"

const int SCREEN_WIDTH = 1024;
const int SCREEN_HEIGHT = 768;

// Function Prototypes
SimulationParams loadSimulation1Parameters();
SimulationParams loadSimulation2Parameters();
SimulationParams loadSimulation3Parameters();
SimulationParams loadSimulation4Parameters();
SimulationParams loadSimulation5Parameters();
SimulationParams loadSimulation6Parameters();
SimulationParams loadSimulation7Parameters();
SimulationParams loadSimulation8Parameters();

int main( int argc, char* args[] )
{
    Display mDisplay;
    Simulation mSimulation;

    if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
    {
        std::cout << "SDL could not initialize! SDL_Error: " <<  SDL_GetError() << std::endl;
        return -1;
    }
    if( TTF_Init() == -1 )
    {
        std::cout << "SDL_ttf could not initialize! SDL_ttf Error: " << TTF_GetError() << std::endl;
        return -1;
    }

    if (!mDisplay.createRenderer("EKF Robot Localization Simulator", SCREEN_WIDTH, SCREEN_HEIGHT)){return -1;}

    mSimulation.reset(loadSimulation1Parameters());
    bool mRunning = true;
    while(mRunning)
    {
        mSimulation.update();

        mDisplay.clearScreen();
        mSimulation.render(mDisplay);
        mDisplay.showScreen();

        SDL_Event event;
        while( SDL_PollEvent( &event ) != 0 )
        {
            if( event.type == SDL_QUIT ){mRunning = false;}
            else if (event.type == SDL_KEYDOWN)
            {
                switch( event.key.keysym.sym )
                {
                    case SDLK_SPACE: mSimulation.togglePauseSimulation(); break;
                    case SDLK_ESCAPE: mRunning = false; break;
                    case SDLK_KP_PLUS: mSimulation.increaseZoom(); break;
                    case SDLK_KP_MINUS: mSimulation.decreaseZoom(); break;
                    case SDLK_RIGHTBRACKET: mSimulation.increaseTimeMultiplier(); break;
                    case SDLK_LEFTBRACKET: mSimulation.decreaseTimeMultiplier(); break;
                    case SDLK_r: mSimulation.reset(); break;
                    case SDLK_1: mSimulation.reset(loadSimulation1Parameters()); break;
                    case SDLK_2: mSimulation.reset(loadSimulation2Parameters()); break;
                    case SDLK_3: mSimulation.reset(loadSimulation3Parameters()); break;
                    case SDLK_4: mSimulation.reset(loadSimulation4Parameters()); break;
                    case SDLK_5: mSimulation.reset(loadSimulation5Parameters()); break;
                    case SDLK_6: mSimulation.reset(loadSimulation6Parameters()); break;
                    case SDLK_7: mSimulation.reset(loadSimulation7Parameters()); break;
                    case SDLK_8: mSimulation.reset(loadSimulation8Parameters()); break;
                }
            }
        }
    }

    mDisplay.destroyRenderer();
    TTF_Quit();
    SDL_Quit();

    return 0;
}

// ============================================================================ //
// SIMULATION PROFILES
// ============================================================================ //

// Profile 1: Straight Line + All Sensors (Simple Test)
SimulationParams loadSimulation1Parameters()
{
    SimulationParams sim;
    sim.profile_name = "1 - Straight Line + All Sensors";
    sim.end_time = 30;
    sim.robot_initial_x = 2.0;
    sim.robot_initial_y = 2.0;
    sim.robot_initial_theta = 0.0;
    sim.robot_commands.emplace_back(new MotionCommandStraight(20.0, 0.5));
    return sim;
}

// Profile 2: Square Path + All Sensors (Turn Test)
SimulationParams loadSimulation2Parameters()
{
    SimulationParams sim;
    sim.profile_name = "2 - Square Path + All Sensors";
    sim.end_time = 80;
    sim.robot_initial_x = 2.0;
    sim.robot_initial_y = 2.0;
    sim.robot_initial_theta = 0.0;
    sim.robot_commands.emplace_back(new MotionCommandStraight(8.0, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandTurnInPlace(M_PI/2.0, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandStraight(8.0, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandTurnInPlace(M_PI, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandStraight(8.0, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandTurnInPlace(-M_PI/2.0, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandStraight(8.0, 0.5));
    return sim;
}

// Profile 3: Waypoint Navigation + All Sensors
SimulationParams loadSimulation3Parameters()
{
    SimulationParams sim;
    sim.profile_name = "3 - Waypoint Navigation + All Sensors";
    sim.end_time = 120;
    sim.robot_initial_x = 2.0;
    sim.robot_initial_y = 2.0;
    sim.robot_initial_theta = 0.0;
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(10, 5, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(10, 10, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(5, 13, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(2, 2, 0.5));
    return sim;
}

// Profile 4: Dead Reckoning Only (No Range Sensor - Shows Drift)
SimulationParams loadSimulation4Parameters()
{
    SimulationParams sim;
    sim.profile_name = "4 - Dead Reckoning Only (No Landmarks)";
    sim.end_time = 120;
    sim.range_sensor_enabled = false;
    sim.robot_initial_x = 2.0;
    sim.robot_initial_y = 2.0;
    sim.robot_initial_theta = 0.0;
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(10, 5, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(10, 10, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(5, 13, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(2, 2, 0.5));
    return sim;
}

// Profile 5: High Noise Sensors
SimulationParams loadSimulation5Parameters()
{
    SimulationParams sim;
    sim.profile_name = "5 - High Sensor Noise";
    sim.end_time = 120;
    sim.encoder_velocity_noise_std = 0.2;
    sim.imu_gyro_noise_std = 0.02;
    sim.range_noise_std = 2.0;
    sim.bearing_noise_std = 0.08;
    sim.robot_initial_x = 2.0;
    sim.robot_initial_y = 2.0;
    sim.robot_initial_theta = 0.0;
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(10, 5, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(10, 10, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(5, 13, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(2, 2, 0.5));
    return sim;
}

// Profile 6: IMU Bias + Encoder Drift
SimulationParams loadSimulation6Parameters()
{
    SimulationParams sim;
    sim.profile_name = "6 - IMU Bias + Encoder Drift";
    sim.end_time = 120;
    sim.imu_gyro_bias = 0.02;
    sim.encoder_drift_rate = 0.005;
    sim.robot_initial_x = 2.0;
    sim.robot_initial_y = 2.0;
    sim.robot_initial_theta = 0.0;
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(5, 5, 0.4));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(15, 3, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(18, 12, 0.4));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(5, 13, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(2, 2, 0.4));
    return sim;
}

// Profile 7: Limited Range + FOV
SimulationParams loadSimulation7Parameters()
{
    SimulationParams sim;
    sim.profile_name = "7 - Limited Range + FOV";
    sim.end_time = 120;
    sim.range_sensor_max_range = 5.0;
    sim.range_sensor_fov = M_PI * 1.5;  // 270 degrees (blind spot behind)
    sim.robot_initial_x = 2.0;
    sim.robot_initial_y = 2.0;
    sim.robot_initial_theta = 0.0;
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(10, 5, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(15, 10, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(10, 13, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(5, 10, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(2, 2, 0.5));
    return sim;
}

// Profile 8: CAPSTONE - All Challenges Combined
SimulationParams loadSimulation8Parameters()
{
    SimulationParams sim;
    sim.profile_name = "8 - CAPSTONE";
    sim.end_time = 180;
    sim.imu_gyro_bias = -0.015;
    sim.encoder_drift_rate = 0.003;
    sim.range_noise_std = 1.0;
    sim.range_sensor_max_range = 6.0;
    sim.range_sensor_fov = M_PI * 1.5;
    sim.robot_initial_x = 3.0;
    sim.robot_initial_y = 3.0;
    sim.robot_initial_theta = M_PI/4.0;
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(5, 5, 0.4));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(10, 2, 0.6));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(15, 5, 0.3));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(18, 10, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(15, 13, 0.4));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(10, 13, 0.6));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(5, 10, 0.4));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(2, 13, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(7, 7, 0.3));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(18, 3, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(18, 12, 0.5));
    sim.robot_commands.emplace_back(new MotionCommandMoveTo(3, 3, 0.4));
    return sim;
}
