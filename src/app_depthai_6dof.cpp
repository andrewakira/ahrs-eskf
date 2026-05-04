#include <iostream>
#include <opencv2/opencv.hpp>
#include "DepthAISensor.hpp"
#include "ImuAttitudeFilter.hpp"
#include "ImuVisualizer.hpp"
#include "ImuIO.hpp"

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    AHRSParams params = AHRSParams::loadFromFile("./data/ahrs_6dof.cfg");
    params.print();

    ImuAttitudeFilter imuAttitudeFilter(params); 
 
    ImuVisualizer viewer;
    DepthAISensor depthAISensor;
    depthAISensor.setImuCallback(
        [&viewer, &imuAttitudeFilter](const DepthAISensor::ImuData& imu) {
            IMU data(
                imu.timestamp,
                Eigen::Vector3d(imu.gyro[0], imu.gyro[1], imu.gyro[2]),
                Eigen::Vector3d(imu.accel[0], imu.accel[1], imu.accel[2])
            );

            static bool isInit = false;
            if (!isInit) {
                isInit = imuAttitudeFilter.initState(data);
            } else {
                imuAttitudeFilter.predict(data);
                imuAttitudeFilter.updateAccel(data);
            }
            
            Eigen::Quaterniond q = imuAttitudeFilter.getQuaternion();
            Eigen::Vector3d bgNominal = imuAttitudeFilter.getBgNominal();

            viewer.pushImu(
                imu.accel[0], imu.accel[1], imu.accel[2],
                imu.gyro[0],  imu.gyro[1],  imu.gyro[2],
                q
            );
        }
    );


    depthAISensor.start();

    while (!viewer.shouldQuit()) {
        viewer.renderOnce();
        this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}