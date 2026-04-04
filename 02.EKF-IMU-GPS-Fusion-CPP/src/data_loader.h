#pragma once

#include "ekf_types.h"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

// ═══════════════════════════════════════════════
//  CSV Data Loader — loads fake or KITTI-format
//  sensor data from CSV files
// ═══════════════════════════════════════════════
class DataLoader {
public:
    // ── Load IMU CSV ──
    //  Format: timestamp,ax,ay,az,gx,gy,gz
    static std::vector<ImuMeasurement> loadImu(const std::string& path) {
        std::vector<ImuMeasurement> data;
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cerr << "[DataLoader] Cannot open IMU file: " << path << "\n";
            return data;
        }

        std::string line;
        std::getline(file, line); // skip header

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream ss(line);
            ImuMeasurement m{};
            char delim;
            ss >> m.timestamp >> delim
               >> m.ax >> delim >> m.ay >> delim >> m.az >> delim
               >> m.gx >> delim >> m.gy >> delim >> m.gz;
            data.push_back(m);
        }

        std::cout << "[DataLoader] Loaded " << data.size() << " IMU measurements\n";
        return data;
    }

    // ── Load GPS CSV ──
    //  Format: timestamp,lat,lon,alt,vn,ve,vd
    static std::vector<GpsMeasurement> loadGps(const std::string& path) {
        std::vector<GpsMeasurement> data;
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cerr << "[DataLoader] Cannot open GPS file: " << path << "\n";
            return data;
        }

        std::string line;
        std::getline(file, line); // skip header

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream ss(line);
            GpsMeasurement m{};
            char delim;
            ss >> m.timestamp >> delim
               >> m.lat >> delim >> m.lon >> delim >> m.alt >> delim
               >> m.vn  >> delim >> m.ve  >> delim >> m.vd;
            m.has_velocity = true;
            data.push_back(m);
        }

        std::cout << "[DataLoader] Loaded " << data.size() << " GPS measurements\n";
        return data;
    }

    // ── Load Ground Truth CSV ──
    //  Format: timestamp,px,py,pz,vx,vy,vz,roll,pitch,yaw
    static std::vector<GroundTruth> loadGroundTruth(const std::string& path) {
        std::vector<GroundTruth> data;
        std::ifstream file(path);
        if (!file.is_open()) {
            std::cerr << "[DataLoader] Cannot open GT file: " << path << "\n";
            return data;
        }

        std::string line;
        std::getline(file, line); // skip header

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream ss(line);
            GroundTruth g{};
            char delim;
            ss >> g.timestamp >> delim
               >> g.px >> delim >> g.py >> delim >> g.pz >> delim
               >> g.vx >> delim >> g.vy >> delim >> g.vz >> delim
               >> g.roll >> delim >> g.pitch >> delim >> g.yaw;
            data.push_back(g);
        }

        std::cout << "[DataLoader] Loaded " << data.size() << " ground truth entries\n";
        return data;
    }
};
