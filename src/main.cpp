#include <iostream>
#include <opencv2/opencv.hpp>
#include "DepthAISensor.hpp"
#include "ImuAttitudeFilter.hpp"
#include "ImuVisualizer.hpp"

using namespace cv;
using namespace std;


int main(int argc, char** argv) {
    ImuVisualizer viewer;
    
    ImuAttitudeFilter imuAttitudeFilter(0.00015702853512975353, 0.014488592710622498, 0.2624646451739678*10, 0.01);
    
    
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


    namedWindow("name");
    while(true) {
        int key = waitKey(33);
        if (key == 'q') {
            break;
        }
    }

    return 0;
}
