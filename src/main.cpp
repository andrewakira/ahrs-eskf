#include <iostream>
#include <opencv2/opencv.hpp>
#include "DepthAISensor.hpp"
//#include "IMUIntegration.hpp"
#include "ImuAttitudeFilter.hpp"
#include "ImuVisualizer.hpp"

using namespace cv;
using namespace std;


int main(int argc, char** argv) {
    Mat frame;
    Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.8); 
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();

    ImuAttitudeFilter imuAttitudeFilter(0.00015702853512975353, 0.014488592710622498, 0.2624646451739678*10);
    DepthAISensor depthAISensor;
    

    ImuVisualizer viewer;
    depthAISensor.setImuCallback(
        [&viewer, &imuAttitudeFilter](const DepthAISensor::ImuData& imu) {
            IMU data(
                imu.timestamp,
                Eigen::Vector3d(imu.gyro[0], imu.gyro[1], imu.gyro[2]),
                Eigen::Vector3d(imu.accel[0], imu.accel[1], imu.accel[2])
            );

            imuAttitudeFilter.predict(data);
            imuAttitudeFilter.updateAccel(data);

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

/*
int main(int argc, char** argv) {
    Mat frame;
    Eigen::Vector3d gravity = Eigen::Vector3d(0, 0, -9.8); 
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    IMUIntegration imuIntegration(gravity, bg, ba);
    DepthAISensor depthAISensor;
    
    depthAISensor.setImuCallback(
        [&imuIntegration](const DepthAISensor::ImuData& imu) {
            
            imuIntegration.AddIMU(IMU(imu.timestamp, Eigen::Vector3d(imu.gyro[0], imu.gyro[1], imu.gyro[2]), Eigen::Vector3d(imu.accel[0], imu.accel[1], imu.accel[2])));
            
            constexpr double RAD2DEG = 180.0 / 3.141592653589793;
            if (fabs(imuIntegration.sumTime - 1) < 1e-2) {
                Sophus::SO3d R = imuIntegration.GetR();
                cout << "sum time : " << imuIntegration.sumTime << endl;
                Eigen::Vector3d w = R.log();
                std::cout << "log(R) degree = " << w.norm() * RAD2DEG << std::endl;
                Eigen::Vector3d p = imuIntegration.GetP();
                std:: cout << "p: " << p.transpose() << endl;

                //Eigen::Vector3d euler = R.matrix().eulerAngles(0,1,2);
                //Eigen::Vector3d euler_deg = euler * RAD2DEG;  
                //cout << "roll pitch yaw: " << euler_deg.transpose() << endl;
            } else if (fabs(imuIntegration.sumTime - 60) < 1e-2) {
                Sophus::SO3d R = imuIntegration.GetR();
                cout << "sum time : " << imuIntegration.sumTime << endl;
                Eigen::Vector3d w = R.log();
                std::cout << "log(R) degree = " << w.norm() * RAD2DEG << std::endl;

                Eigen::Vector3d p = imuIntegration.GetP();
                std:: cout << "p: " << p.transpose() << endl;
            }

        }
    );
    
    depthAISensor.start();

    namedWindow("name");
    while(true) {
        int key = waitKey(33);
        if (key == 'q') {
            break;
        }
    }

    return 0;
}
*/
