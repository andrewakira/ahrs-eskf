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
    ImuAttitudeFilter(double sigmaGyroBiasNoise, double sigmaGyroNoise, double sigmaAcceNoise);
    ~ImuAttitudeFilter();

    void staticCalib(const IMU& imu);
    void predict(const IMU& imu);
    bool updateAccel(const IMU& imu);
    void reset();

    Eigen::Quaterniond getQuaternion();
    Eigen::Vector3d getBgNominal();

    static Eigen::Quaterniond rotvecToQuat(const Eigen::Vector3d& rotvec);
    static Eigen::Matrix3d rotvecToMatrix(const Eigen::Vector3d& rotvec);
    static Eigen::Matrix3d skew(const Eigen::Vector3d& v);
    static Eigen::Matrix3d rightJacobianSO3(const Eigen::Vector3d& theta);
    
private:
    void update(const Eigen::Vector3d& z, const Eigen::Vector3d& bRef, const double stdMeasure);
    void resetErrorState();


    IMU lastImu;
    bool hasLastImu = false;


    Eigen::Quaterniond qNominal;
    Eigen::Vector3d bgNominal;

    //Eigen::Matrix<double, 6, 6> Fx;
    Eigen::Matrix<double, 6, 6> P;

    Eigen::Matrix<double, 6, 6> Q;
    //Eigen::Matrix<double, 6, 6> G;
    Eigen::Matrix<double, 3, 3> RN;
    //Eigen::Matrix<double, 3, 6> H;
    //Eigen::Matrix<double, 6, 3> K;

    double sigmaGyroBiasNoise;      // rad/s
    double sigmaGyroNoise;          // rad/(s*sqrt(s))
    double sigmaAcceNoise;
    double gravityNorm = 9.81;
    double accelGate = 0.5;


};



#endif
