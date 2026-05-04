#ifndef common_H
#define common_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

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

#endif
