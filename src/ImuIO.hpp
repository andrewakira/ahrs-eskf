#ifndef ImuIO_h
#define ImuIO_h

#include <iostream>
#include <fstream>
#include <iomanip>
#include <Eigen/Core>

#include <sstream>
#include <vector>
#include <string>

class ImuCsvLogger {
public:
    explicit ImuCsvLogger(std::string path) {
        file.open(path);
        if (!file.is_open()) {
            std::cerr << "[ImuCsvLogger] Failed to open file: " << path << std::endl;
            return;
        }
        
    }

    ~ImuCsvLogger() {
        if (file.is_open()) {
            file.close();
        }
    }

    void log(double t, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc) {
        if (!file.is_open()) {
            return;
        }
        static long int n = 0;
        if (n == 0) {
            file << "t,gx,gy,gz,ax,ay,az\n";
        }
        

        file << std::fixed << std::setprecision(9)
            << t << ","
            << gyro.x() << ","
            << gyro.y() << ","
            << gyro.z() << ","
            << acc.x() << ","
            << acc.y() << ","
            << acc.z() << "\n";
        n++;
    }

    void logRotation(const Eigen::Quaterniond& q, const Eigen::Vector3d& euler, double gravity_error, const Eigen::Quaterniond& initq) {
        if (!file.is_open()) {
            return;
        }

        if (countLine == 0) {
            file << "roll,pitch,yaw,angle_x,angle_y,angle_z,gravity_error,w,x,y,z,gt_error\n";
        }

        auto angleDeg = [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
            double c = a.normalized().dot(b.normalized());
            c = std::clamp(c, -1.0, 1.0);   // 避免 acos nan
            return std::acos(c) * 180.0 / M_PI;
        };
        Eigen::Matrix3d R = q.normalized().toRotationMatrix();
        Eigen::Vector3d origin(0, 0, 0);
        Eigen::Vector3d xAxis = R * Eigen::Vector3d(1, 0, 0);
        Eigen::Vector3d yAxis = R * Eigen::Vector3d(0, 1, 0);
        Eigen::Vector3d zAxis = R * Eigen::Vector3d(0, 0, 1);

        double angle_x = angleDeg(Eigen::Vector3d(1, 0, 0), xAxis);
        double angle_y = angleDeg(Eigen::Vector3d(0, 1, 0), yAxis);
        double angle_z = angleDeg(Eigen::Vector3d(0, 0, 1), zAxis);

        Eigen::Matrix3d R_gt = initq.normalized().toRotationMatrix();//Eigen::Matrix3d::Identity();
        Eigen::Matrix3d R_err = R_gt.transpose() * R;
        double angle_err_deg = Eigen::AngleAxisd(R_err).angle() * 180.0 / M_PI;

        if( countLine % 5 == 0) {
            Eigen::Quaterniond qq = q;
            if (qq.w() < 0) {
                qq.coeffs() *= -1.0; 
            }
            file << std::fixed << std::setprecision(5)
                << euler[0] << ","
                << euler[1] << ","
                << euler[2] << ","
                << angle_x << ","
                << angle_y << ","
                << angle_z << ","
                << gravity_error << ","
                << qq.w() << ","
                << qq.x() << ","
                << qq.y() << ","
                << qq.z() << ","
                << angle_err_deg << "\n";
        }


        if (countLine % 200 == 0) {
            std::cout << " roll  = " << euler[0];
            std::cout << " pitch = " << euler[1];
            std::cout << " yaw   = " << euler[2];
            std::cout << " gravity_error = " << gravity_error << std::endl;


            Eigen::Quaterniond qq = q;
            if (qq.w() < 0) {
                qq.coeffs() *= -1.0; 
            }
            std::cout << "q[wxyz] = " << qq.w() << " " << qq.x() << " " << qq.y() << " " << qq.z() << std::endl;
        }

        countLine++;
    }

    void logRotation2(const Eigen::Quaterniond& q_in,
                 const Eigen::Vector3d& euler,
                 double gravity_error,
                 const Eigen::Vector3d& unobs_axis_body){
        if (!file.is_open()) {
            return;
        }

        static long int n = 0;
        static bool wroteHeader = false;
        static bool hasPrevQ = false;
        static Eigen::Quaterniond qPrev;
        static double cumUnobsDeg = 0.0;

        if (!wroteHeader) {
            file << "roll,pitch,yaw,gravity_error,"
                << "qw,qx,qy,qz,"
                << "rel_angle_deg,unobs_delta_deg,obs_delta_deg,cum_unobs_deg,"
                << "unobs_x,unobs_y,unobs_z\n";
            wroteHeader = true;
        }

        Eigen::Quaterniond q = q_in.normalized();

        if (hasPrevQ && qPrev.dot(q) < 0.0) {
            q.coeffs() *= -1.0;
        }

        double relAngleDeg = 0.0;
        double unobsDeltaDeg = 0.0;
        double obsDeltaDeg = 0.0;

        if (hasPrevQ) {
            Eigen::Quaterniond dq = qPrev.conjugate() * q;
            dq.normalize();

            if (dq.w() < 0.0) {
                dq.coeffs() *= -1.0;
            }

            Eigen::Vector3d v(dq.x(), dq.y(), dq.z());
            double vNorm = v.norm();

            Eigen::Vector3d dtheta = Eigen::Vector3d::Zero();

            if (vNorm < 1e-12) {
                dtheta = 2.0 * v;
            } else {
                double angle = 2.0 * std::atan2(vNorm, dq.w());
                dtheta = angle * v / vNorm;
            }

            Eigen::Vector3d u = unobs_axis_body.normalized();

            double unobsRad = dtheta.dot(u);
            Eigen::Vector3d obs = dtheta - unobsRad * u;

            relAngleDeg = dtheta.norm() * 180.0 / M_PI;
            unobsDeltaDeg = unobsRad * 180.0 / M_PI;
            obsDeltaDeg = obs.norm() * 180.0 / M_PI;

            cumUnobsDeg += unobsDeltaDeg;
        }

        qPrev = q;
        hasPrevQ = true;

        if (n % 5 == 0) {
            Eigen::Vector3d u = unobs_axis_body.normalized();

            file << std::fixed << std::setprecision(6)
                << euler[0] << ","
                << euler[1] << ","
                << euler[2] << ","
                << gravity_error << ","
                << q.w() << ","
                << q.x() << ","
                << q.y() << ","
                << q.z() << ","
                << relAngleDeg << ","
                << unobsDeltaDeg << ","
                << obsDeltaDeg << ","
                << cumUnobsDeg << ","
                << u.x() << ","
                << u.y() << ","
                << u.z() << "\n";
        }

        if (n % 200 == 0) {
            std::cout << "roll = " << euler[0]
                    << " pitch = " << euler[1]
                    << " yaw = " << euler[2]
                    << " gravity_error = " << gravity_error
                    << " rel = " << relAngleDeg
                    << " unobs = " << unobsDeltaDeg
                    << " obs = " << obsDeltaDeg
                    << " cum_unobs = " << cumUnobsDeg
                    << std::endl;

            std::cout << "q[wxyz] = "
                    << q.w() << " "
                    << q.x() << " "
                    << q.y() << " "
                    << q.z() << std::endl;
        }

        n++;
    }

private:
    std::ofstream  file;
    long int countLine = 0;
};


class ImuIO {
public:
    static std::vector<std::string> splitTab(const std::string& line) {
        std::vector<std::string> tokens;
        std::stringstream ss(line);
        std::string item;

        while (std::getline(ss, item, '\t')) {
            if (!item.empty() && item[0] == ' ')
                item.erase(0, item.find_first_not_of(" "));
            tokens.push_back(item);
        }
        return tokens;
    }

    static std::vector<std::string> splitComma(const std::string& line) {
        std::vector<std::string> out;
        std::stringstream ss(line);
        std::string item;
        while (std::getline(ss, item, ',')) {
            out.push_back(item);
        }
        return out;
    }

    static bool loadWitmotionCSV(const std::string& filename, std::vector<IMU>& logs) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Cannot open file: " << filename << std::endl;
            return false;
        }

        double timestamp = 0;

        std::string line;

        // skip header
        std::getline(file, line);

        while (std::getline(file, line)) {
            if (line.empty()) {
                continue;
            }

            std::vector<std::string> t = splitComma(line);


            if (t.size() < 32) {
                continue;
            }

            IMU d;

            d.timestamp = timestamp;
            d.accel = Eigen::Vector3d(std::stod(t[3]), std::stod(t[4]), std::stod(t[5]))*9.81;
            d.gyro = Eigen::Vector3d(std::stod(t[6]), std::stod(t[7]), std::stod(t[8]))* M_PI / 180.0;
            d.angle = Eigen::Vector3d(std::stod(t[9]), std::stod(t[10]), std::stod(t[11]));
            d.mag = Eigen::Vector3d(std::stod(t[12]), std::stod(t[13]), std::stod(t[14]));
            d.q = Eigen::Quaterniond(std::stod(t[27]), std::stod(t[28]), std::stod(t[29]), std::stod(t[30]));

            logs.push_back(d);

            timestamp+=0.005;
        }

        file.close();
        return true;
    }

    static void printIMU(const IMU& d) {
        std::cout << "t : " << d.timestamp << std::endl;
        std::cout << "accel : " << d.accel.transpose() << std::endl;
        std::cout << "gyro : " << d.gyro.transpose() << std::endl;
        std::cout << "mag : " << d.mag.transpose() << std::endl;
        std::cout << "angle : " << d.angle.transpose() << std::endl;
        std::cout << "q : " << d.q.w() << " " << d.q.x() << " " << d.q.y() << " " << d.q.z() << std::endl;
    }
};

#endif