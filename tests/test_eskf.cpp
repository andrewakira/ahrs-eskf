#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "ImuAttitudeFilter.hpp"

//rotvecToQuat
TEST(rotvecToQuat, NormalRotation) {
    Eigen::Vector3d rotvec(0, 0, M_PI / 2.0);
    Eigen::Quaterniond q = ImuAttitudeFilter::rotvecToQuat(rotvec);

    EXPECT_NEAR(q.w(), std::cos(M_PI / 4.0), 1e-7);
    EXPECT_NEAR(q.x(), 0.0, 1e-7);
    EXPECT_NEAR(q.y(), 0.0, 1e-7);
    EXPECT_NEAR(q.z(), std::sin(M_PI / 4.0), 1e-7);
    EXPECT_NEAR(q.norm(), 1.0, 1e-7);
}

TEST(rotvecToQuat, SmallAngleApproximation) {
    Eigen::Vector3d rotvec(1e-13, 0, 0); 
    Eigen::Quaterniond q = ImuAttitudeFilter::rotvecToQuat(rotvec);

    EXPECT_NEAR(q.w(), 1.0, 1e-15);
    EXPECT_NEAR(q.x(), 0.5e-13, 1e-15);
    EXPECT_NEAR(q.y(), 0.0, 1e-15);
    EXPECT_NEAR(q.z(), 0.0, 1e-15);
}

//rotvecToMatrix
TEST(rotvecToMatrix, ZAxis90Deg) {
    Eigen::Vector3d w(0, 0, M_PI / 2.0);
    Eigen::Matrix3d R = ImuAttitudeFilter::rotvecToMatrix(w);

    Eigen::Matrix3d R_expected;
    R_expected << 0, -1, 0,
                  1,  0, 0,
                  0,  0, 1;

    EXPECT_TRUE(R.isApprox(R_expected, 1e-12));
}

TEST(rotvecToMatrix, XAxis180Deg) {
    Eigen::Vector3d w(M_PI, 0, 0);
    Eigen::Matrix3d R = ImuAttitudeFilter::rotvecToMatrix(w);

    Eigen::Matrix3d R_expected;
    R_expected << 1,  0,  0,
                  0, -1,  0,
                  0,  0, -1;

    EXPECT_TRUE(R.isApprox(R_expected, 1e-12));
}

TEST(rotvecToMatrix, SmallAngleReturnsIdentity) {
    Eigen::Vector3d w(1e-10, 2e-10, -1e-10);
    Eigen::Matrix3d R = ImuAttitudeFilter::rotvecToMatrix(w);

    Eigen::Matrix3d Rgt;
    Rgt << 1, 1e-10, 2e-10, -1e-10, 1, -1e-10, -2e-10, 1e-10, 1;
    
    EXPECT_TRUE(R.isApprox(Rgt, 1e-12));
}

TEST(rotvecToMatrix, ValidRotationMatrix) {
    Eigen::Vector3d w(0.3, -0.2, 0.5);
    Eigen::Matrix3d R = ImuAttitudeFilter::rotvecToMatrix(w);

    EXPECT_TRUE((R.transpose() * R).isApprox(Eigen::Matrix3d::Identity(), 1e-12));
    EXPECT_NEAR(R.determinant(), 1.0, 1e-12);
}

//rightJacobianSO3
TEST(rightJacobianSO3, ZeroVector) {
    Eigen::Vector3d theta = Eigen::Vector3d::Zero();
    Eigen::Matrix3d Jr = ImuAttitudeFilter::rightJacobianSO3(theta);
    EXPECT_TRUE(Jr.isApprox(Eigen::Matrix3d::Identity(), 1e-15));
}

TEST(rightJacobianSO3, TypicalAngle) {
    Eigen::Vector3d theta(M_PI / 2.0, 0, 0);
    Eigen::Matrix3d Jr = ImuAttitudeFilter::rightJacobianSO3(theta);

    // A = (1 - cos(pi/2)) / (pi/2)^2 = 1 / (pi^2 / 4) = 4 / pi^2
    // B = (pi/2 - sin(pi/2)) / (pi/2)^3 = (pi/2 - 1) / (pi^3 / 8)
    double angle = M_PI / 2.0;
    double A = (1.0 - 0.0) / (angle * angle);
    double B = (angle - 1.0) / (angle * angle * angle);
    
    Eigen::Matrix3d skew_v = ImuAttitudeFilter::skew(theta);
    Eigen::Matrix3d expected = Eigen::Matrix3d::Identity() - A * skew_v + B * skew_v * skew_v;

    EXPECT_TRUE(Jr.isApprox(expected, 1e-12));
}

