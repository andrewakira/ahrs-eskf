#ifndef common_H
#define common_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sophus/se2.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

/*
using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;
using Vec3b = Eigen::Matrix<char, 3, 1>;

using Vec2d = Eigen::Vector2d;
using Vec2f = Eigen::Vector2f;
using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;
using Vec4d = Eigen::Vector4d;
using Vec4f = Eigen::Vector4f;
using Vec5d = Eigen::Matrix<double, 5, 1>;
using Vec5f = Eigen::Matrix<float, 5, 1>;
using Vec6d = Eigen::Matrix<double, 6, 1>;
using Vec6f = Eigen::Matrix<float, 6, 1>;
using Vec9d = Eigen::Matrix<double, 9, 1>;
using Vec15d = Eigen::Matrix<double, 15, 1>;
using Vec18d = Eigen::Matrix<double, 18, 1>;
*/

template <typename T>
struct NavState {
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using SO3 = Sophus::SO3<T>;
    using SE3 = Sophus::SE3<T>;

    NavState() = default;

    // from time, R, p, v, bg, ba
    explicit NavState(double time, const SO3& R = SO3(), const Vec3& t = Vec3::Zero(), const Vec3& v = Vec3::Zero(),
                      const Vec3& bg = Vec3::Zero(), const Vec3& ba = Vec3::Zero())
        : timestamp(time), R(R), p(t), v(v), bg(bg), ba(ba) {

        }

    // from pose and vel
    NavState(double time, const SE3& pose, const Vec3& vel = Vec3::Zero())
        : timestamp(time), R(pose.so3()), p(pose.translation()), v(vel) {

        }

    Sophus::SE3<T> GetSE3() const { return SE3(R, p); }

    friend std::ostream& operator<<(std::ostream& os, const NavState<T>& s) {
        os << "p: " << s.p.transpose() << ", v: " << s.v.transpose()
           << ", q: " << s.R.unit_quaternion().coeffs().transpose() << ", bg: " << s.bg.transpose()
           << ", ba: " << s.ba.transpose();
        return os;
    }

    double timestamp = 0;    
    SO3 R;                   
    Vec3 p = Vec3::Zero();   
    Vec3 v = Vec3::Zero();   
    Vec3 bg = Vec3::Zero();  
    Vec3 ba = Vec3::Zero();
};

using NavStated = NavState<double>;
using NavStatef = NavState<float>;

struct IMU {
    IMU() = default;
    IMU(double t, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acce) 
    : timestamp(t), gyro(gyro), acce(acce) {}

    double timestamp = 0.0;
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    Eigen::Vector3d acce = Eigen::Vector3d::Zero();
};

#endif