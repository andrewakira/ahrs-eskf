#include <iostream>
#include "ImuAttitudeFilter.hpp"
#include "ImuVisualizer.hpp"
#include "ImuIO.hpp"

using namespace std;

int main() {
    ImuVisualizer viewer;
    ImuAttitudeFilter imuAttitudeFilter(ImuAttitudeFilter::FusionMode::Imu9Axis, 0.00015702853512975353, 0.014488592710622498, 0.262, 0.262/3, 0.01);

    std::vector<IMU> logs;
    ImuIO::loadWitmotionCSV("./data/witmotion_dynamic0.csv", logs);
    cout << "Total : " << logs.size() << endl;
    ImuIO::printIMU(logs[0]);

    for(int i = 0; i < logs.size(); i++) {
        IMU data = logs[i];
        static bool isInit = false;
        if (!isInit) {
            isInit = imuAttitudeFilter.initState(data);
        } else {
            imuAttitudeFilter.predict(data);
            imuAttitudeFilter.updateAccel(data);
            imuAttitudeFilter.updateMag(data);
        }
        
        Eigen::Quaterniond q = imuAttitudeFilter.getQuaternion();
        Eigen::Vector3d bgNominal = imuAttitudeFilter.getBgNominal();

        viewer.pushImu(
            data.accel[0], data.accel[1], data.accel[2],
            data.gyro[0],  data.gyro[1], data.gyro[2],
            q
        );
        
        if (viewer.shouldQuit()) {
            break;
        } else {
            viewer.renderOnce();
            this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        if (i % 10 == 0) {
            cout << "[" + to_string(i) + "]---" << endl;
            cout << "q_est : " << q.w() << " " << q.x() <<" " << q.y() << " " << q.z() << endl;
            cout << "q_gt : " << data.q.w() << " " << data.q.x() <<" " << data.q.y() << " " << data.q.z() << endl;

            Eigen::Vector3d euler = ImuIO::quatToWitionEulerDegree(q);
            cout << "angle_est : " << euler[0] << " " << euler[1] << " " << euler[2] << endl;
            cout << "angle_gt : " << data.angle.transpose() << endl;
        }
    }


    return 0;
}
