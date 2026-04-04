cmake_minimum_required(VERSION 3.10)
project(EKF_IMU_GPS_Fusion VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# ── EKF Fusion executable ──
add_executable(ekf_imu_gps_fusion
    src/main.cpp
)

target_include_directories(ekf_imu_gps_fusion PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(ekf_imu_gps_fusion PRIVATE
    Eigen3::Eigen
)

# ── Fake data generator ──
add_executable(generate_fake_data
    src/generate_fake_data.cpp
)

target_include_directories(generate_fake_data PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(generate_fake_data PRIVATE
    Eigen3::Eigen
)
