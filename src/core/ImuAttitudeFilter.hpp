#ifndef ImuAttitudeFilter_H
#define ImuAttitudeFilter_H

#include "common.hpp"
#include <iostream>
#include <vector>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>

class ImuAttitudeFilter  {
public:
    explicit ImuAttitudeFilter(const AHRSParams& params);
    explicit ImuAttitudeFilter(const std::string paramsPath);
    ~ImuAttitudeFilter();

    bool initState(const IMU& imu);
    void predict(const IMU& imu);
    bool updateAccel(const IMU& imu);
    bool updateMag(const IMU& imu);

    Eigen::Quaterniond getQuaternion();
    Eigen::Vector3d getBgNominal();

    static Eigen::Quaterniond rotvecToQuat(const Eigen::Vector3d& rotvec);
    static Eigen::Matrix3d rotvecToMatrix(const Eigen::Vector3d& rotvec);
    static Eigen::Matrix3d skew(const Eigen::Vector3d& v);
    static Eigen::Matrix3d rightJacobianSO3(const Eigen::Vector3d& theta);
    
private:
    void update(const Eigen::Vector3d& z, const Eigen::Vector3d& bRef, const double stdMeasure);

    IMU lastImu;
    bool hasLastImu = false;

    AHRSParams params;

    Eigen::Quaterniond qNominal = Eigen::Quaterniond::Identity();
    Eigen::Vector3d bgNominal = Eigen::Vector3d::Zero();

    Eigen::Matrix<double, 6, 6> P;

    // ENU: x=east, y=north, z=up
    const Eigen::Vector3d gravityRefWorld = Eigen::Vector3d(0, 0, 1);
    const Eigen::Vector3d magRefWorld = Eigen::Vector3d(0, 1, 0);

    int initCount = 0;
    Eigen::Vector3d accelMean = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyroMean = Eigen::Vector3d::Zero();
    Eigen::Vector3d magMean = Eigen::Vector3d::Zero();
};



#endif
