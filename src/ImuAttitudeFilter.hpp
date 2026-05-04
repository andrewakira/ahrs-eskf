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
    enum class FusionMode {
        Imu6Axis,
        Imu9Axis
    };
    ImuAttitudeFilter(FusionMode mode, double sigmaGyroBiasNoise, double sigmaGyroNoise, double sigmaAccelNoise, double sigmaMagNoise, double initialCovarianceStd);
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

    FusionMode mode;
    IMU lastImu;
    bool hasLastImu = false;


    Eigen::Quaterniond qNominal;
    Eigen::Vector3d bgNominal;

    Eigen::Matrix<double, 6, 6> P;

    double sigmaGyroBiasNoise;      // rad/s
    double sigmaGyroNoise;          // rad/(s*sqrt(s))
    double sigmaAccelNoise;
    double sigmaMagNoise;
    double gravityNorm = 9.81;
    double accelGate = 0.5;
    double gyroGate = 0.1; // 5 degree/s

    // ENU: x=east, y=north, z=up
    const Eigen::Vector3d gravityRefWorld = Eigen::Vector3d(0, 0, 1);
    const Eigen::Vector3d magRefWorld = Eigen::Vector3d(0, 1, 0);

    const int initMaxCount = 10;
    int initCount = 0;
    Eigen::Vector3d accelMean;
    Eigen::Vector3d gyroMean;
    Eigen::Vector3d magMean;
};



#endif
