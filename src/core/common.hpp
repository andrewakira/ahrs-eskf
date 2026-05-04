#ifndef common_H
#define common_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unordered_map>
#include <sstream>
#include <iomanip>

struct IMU {
    IMU() = default;
    IMU(double t, const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel) 
    : timestamp(t), gyro(gyro), accel(accel) {
    }

    IMU(double t, const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, const Eigen::Vector3d& mag) 
    : timestamp(t), gyro(gyro), accel(accel), mag(mag) {
    }

    IMU(double t, const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, const Eigen::Vector3d& mag, const Eigen::Vector3d& angle, const Eigen::Quaterniond q) 
    : timestamp(t), gyro(gyro), accel(accel), mag(mag), angle(angle), q(q) {
    }

    double timestamp = 0.0;
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d mag = Eigen::Vector3d::Zero();
    Eigen::Vector3d angle = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q;
};

struct AHRSParams {
    enum class FusionMode {
        Imu6Axis,
        Imu9Axis
    };

    FusionMode fusionMode = FusionMode::Imu6Axis;

    double stdGyroBiasNoise = 0.00015702853512975353; // rad/s
    double stdGyroNoise     = 0.014488592710622498;   // rad/(s*sqrt(s))
    double stdAccelNoise    = 0.262;
    double stdMagNoise      = 0.262 / 3.0;
    double stdInitialCovariance = 0.01;

    double gravityNorm = 9.81;
    double accelGate   = 0.5;
    double gyroGate    = 0.1; // 5 degree/s
    double lowMagGate = 15;
    double highMagGate = 80;
    int initMaxCount = 10;

    static AHRSParams loadFromFile(const std::string& path) {
        AHRSParams p;

        std::ifstream fin(path);
        if (!fin.is_open()) {
            std::cout << "Cannot open config : " << path << std::endl;
            return p;
        }

        std::string line;
        std::unordered_map<std::string, double> kv;

        while (std::getline(fin, line)) {
            if (line.empty() || line[0] == '#') {
                continue;
            }

            std::stringstream ss(line);
            std::string key, value;

            if (std::getline(ss, key, '=') &&
                std::getline(ss, value)) {
                kv[key] = std::stod(value);
            }
        }

        if (kv.count("fusionMode"))       p.fusionMode       = static_cast<FusionMode>(static_cast<int>(kv["fusionMode"]));
        if (kv.count("stdGyroBiasNoise")) p.stdGyroBiasNoise = kv["stdGyroBiasNoise"];
        if (kv.count("stdGyroNoise"))     p.stdGyroNoise     = kv["stdGyroNoise"];
        if (kv.count("stdAccelNoise"))    p.stdAccelNoise    = kv["stdAccelNoise"];
        if (kv.count("stdMagNoise"))      p.stdMagNoise      = kv["stdMagNoise"];
        if (kv.count("stdInitialCovariance")) p.stdInitialCovariance = kv["stdInitialCovariance"];
        if (kv.count("gravityNorm"))        p.gravityNorm        = kv["gravityNorm"];
        if (kv.count("accelGate"))          p.accelGate          = kv["accelGate"];
        if (kv.count("gyroGate"))           p.gyroGate           = kv["gyroGate"];
        if (kv.count("lowMagGate"))         p.lowMagGate         = kv["lowMagGate"];
        if (kv.count("highMagGate"))        p.highMagGate        = kv["highMagGate"];
        if (kv.count("initMaxCount"))       p.initMaxCount       = static_cast<int>(kv["initMaxCount"]);


        return p;
    }

    void print() const {
        std::cout << "========== AHRS Params ==========" << std::endl;
        std::cout << "fusionMode          : " 
                << (fusionMode == FusionMode::Imu6Axis ? "Imu6Axis" : "Imu9Axis") << std::endl;

        std::cout << "stdGyroBiasNoise    : " << toStringPrec(stdGyroBiasNoise) << std::endl;
        std::cout << "stdGyroNoise        : " << toStringPrec(stdGyroNoise) << std::endl;
        std::cout << "stdAccelNoise       : " << toStringPrec(stdAccelNoise) << std::endl;
        std::cout << "stdMagNoise         : " << toStringPrec(stdMagNoise) << std::endl;
        std::cout << "stdInitialCovariance: " << toStringPrec(stdInitialCovariance) << std::endl;

        std::cout << "gravityNorm         : " << gravityNorm << std::endl;
        std::cout << "accelGate           : " << accelGate << std::endl;
        std::cout << "gyroGate            : " << gyroGate << std::endl;
        std::cout << "lowMagGate          : " << lowMagGate << std::endl;
        std::cout << "highMagGate         : " << highMagGate << std::endl;

        std::cout << "initMaxCount        : " << initMaxCount << std::endl;
        std::cout << "=================================" << std::endl;
    }

    static std::string toStringPrec(double value, int prec = 15) {
        std::ostringstream oss;
        oss << std::setprecision(prec) << value;
        return oss.str();
    }
};
#endif
