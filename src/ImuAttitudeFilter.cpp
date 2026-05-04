#include "ImuAttitudeFilter.hpp"

ImuAttitudeFilter::ImuAttitudeFilter(FusionMode mode, double sigmaGyroBiasNoise, double sigmaGyroNoise, double sigmaAccelNoise, double sigmaMagNoise, double initialCovarianceStd) {
    this->mode = mode;    

    this->sigmaGyroBiasNoise = sigmaGyroBiasNoise;
    this->sigmaGyroNoise = sigmaGyroNoise;
    this->sigmaAccelNoise = sigmaAccelNoise;  
    this->sigmaMagNoise = sigmaMagNoise;

    this->qNominal = Eigen::Quaterniond::Identity();
    this->bgNominal = Eigen::Vector3d::Zero();
    this->P = Eigen::Matrix<double, 6, 6>::Identity() * (initialCovarianceStd * initialCovarianceStd);
    this->hasLastImu = false; 
}

ImuAttitudeFilter::~ImuAttitudeFilter() {
}

bool ImuAttitudeFilter::initState(const IMU& imu) {
    if (initCount >= initMaxCount) {
        return true;
    }
     
    double aNorm = imu.acce.norm();
    if (std::abs(aNorm - gravityNorm) > accelGate) {
        initCount = 0;
        accelMean.setZero();
        gyroMean.setZero();
        return false;
    }

    if (imu.gyro.norm() > gyroGate) {
        initCount = 0;
        accelMean.setZero();
        gyroMean.setZero();
        return false;
    }

    if (mode == FusionMode::Imu9Axis) {
        if (imu.mag.norm() < 5 || imu.mag.norm() > 80) {
            initCount = 0;
            accelMean.setZero();
            gyroMean.setZero();
            magMean.setZero();
            return false;
        }
    }

    const Eigen::Vector3d a = imu.acce / aNorm;
    accelMean += a / initMaxCount;
    gyroMean += imu.gyro / initMaxCount;
    if (mode == FusionMode::Imu9Axis) {
        magMean += imu.mag / initMaxCount;
    }
    initCount++;

    if (initCount >= initMaxCount) {
        if (mode == FusionMode::Imu6Axis) {
            // Beacause accelMean - R^T gravityRefWorld = 0, so we need to find R by Quaterniond::FromTwoVectors
            Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(accelMean, gravityRefWorld);
            qNominal = q.normalized();
            bgNominal = gyroMean;
            return true;
        } else if (mode == FusionMode::Imu9Axis) {
            Eigen::Vector3d upB = accelMean.normalized();
            Eigen::Vector3d mB = magMean.normalized();

            Eigen::Vector3d eastB = mB.cross(upB);
            eastB.normalize();

            Eigen::Vector3d northB = upB.cross(eastB).normalized();

            Eigen::Matrix3d Rw2b;
            Rw2b.col(0) = eastB;
            Rw2b.col(1) = northB;
            Rw2b.col(2) = upB;

            Eigen::Matrix3d Rb2w = Rw2b.transpose();
            qNominal = Eigen::Quaterniond(Rb2w).normalized();
            bgNominal = gyroMean;
            return true;
        }
    }
    
    return false;
}

void ImuAttitudeFilter::predict(const IMU& imu) {
    if (!hasLastImu) {
        lastImu = imu;
        hasLastImu = true;
        return;
    }

    const double dt = imu.timestamp - lastImu.timestamp;
    if (dt > 1 || dt  < 0) {
        std::cout << "dt > 1 || dt < 0" << std::endl;
        lastImu = imu;
        return;
    }

    //nominal state prediction
    const Eigen::Vector3d dtheta = (imu.gyro - bgNominal)* dt;
    qNominal = qNominal * rotvecToQuat(dtheta);
    qNominal = qNominal.normalized();

    //covariance prediction
    //Fx = [ R^T{dtheta} | -dt*I]
    //     [ 0           |     I]
    Eigen::Matrix<double, 6, 6> Fx = Eigen::Matrix<double, 6, 6>::Identity();
    Fx.block<3, 3>(0, 0) = rotvecToMatrix(dtheta).transpose();
    Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * -dt;
    //Q = [ dt^2 * sigmaGyroNoise^2 * I |                             0] 
    //    [ 0                           | dt * sigmaGyroBiasNoise^2 * I]
    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Identity();
    Q.block<3, 3>(0, 0) *= (dt * dt * sigmaGyroNoise * sigmaGyroNoise) ;
    Q.block<3, 3>(3, 3) *= (dt * sigmaGyroBiasNoise * sigmaGyroBiasNoise);

    P = Fx*P*Fx.transpose() + Q;

    lastImu = imu;
}

void ImuAttitudeFilter::update(const Eigen::Vector3d& z, const Eigen::Vector3d& bRef, const double measureStd) {
    Eigen::Matrix3d  Rq = qNominal.toRotationMatrix();
    Eigen::Vector3d zHat = Rq.transpose() * bRef;
    const Eigen::Vector3d r = z - zHat;

    // 1. Kalman Gain K
    // H = [ [R^T{q} bRef]x   0 ]
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H.block<3, 3>(0, 0) = skew(zHat);
    H.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();

    //K = PH^T(HPH^T+R)^{-1}
    const Eigen::Matrix3d Rmeas = Eigen::Matrix3d::Identity() * (measureStd * measureStd);
    const Eigen::Matrix3d S = H * P * H.transpose() + Rmeas;
    const Eigen::Matrix<double, 6, 3> K = P * H.transpose() * S.inverse();

    // 2. inject new error state into nominal state
    const Eigen::Matrix<double, 6, 1> deltaX = K * r;
    const Eigen::Vector3d deltaTheta = deltaX.block<3, 1>(0, 0);
    const Eigen::Vector3d deltabg    = deltaX.block<3, 1>(3, 0);
   
    qNominal = (qNominal * rotvecToQuat(deltaTheta)).normalized();
    bgNominal += deltabg;


    // 3. covariance update
    // Joseph form P = (I-KH)P(I-KH)^T +KRK^T
    const Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 6, 6> IKH = I - K * H;
    P = IKH * P * IKH.transpose() + K * Rmeas * K.transpose();

    // 4. reset
    // reset error state //deltaX = 0;
    // reset covariance: P <- G P G^T
    Eigen::Matrix<double, 6, 6> G = Eigen::Matrix<double, 6, 6>::Identity();
    G.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() - 0.5 * skew(deltaTheta);
    P = G * P * G.transpose();
}

bool ImuAttitudeFilter::updateAccel(const IMU& imu) {
    if (!hasLastImu) {
        return false;
    }

    double aNorm = imu.acce.norm();
    if (std::abs(aNorm - gravityNorm) > accelGate) {
        return false;
    }

    const Eigen::Vector3d z = imu.acce / aNorm;

    update(z, gravityRefWorld, sigmaAccelNoise);
    return true;
}


bool ImuAttitudeFilter::updateMag(const IMU& imu) {
    if (mode != FusionMode::Imu9Axis) {
        throw std::logic_error("Make sure fusion mode is setting on Imu9axis");
    }

    if (!hasLastImu) {
        return false;
    }

    double mNorm = imu.mag.norm();
    if (mNorm < 15.0 || mNorm > 80.0) {
        return false;
    }

    Eigen::Vector3d mB = imu.mag / mNorm;

    Eigen::Matrix3d Rq = qNominal.toRotationMatrix();

    // world up expressed in body frame
    Eigen::Vector3d upB = Rq.transpose() * gravityRefWorld;
    upB.normalize();

    // east = m x up
    Eigen::Vector3d eastB = mB.cross(upB);
    double eastNorm = eastB.norm();
    if (eastNorm < 1e-6) {
        return false;
    }
    eastB.normalize();

    // north = up x east
    Eigen::Vector3d northB = upB.cross(eastB);
    double northNorm = northB.norm();
    if (northNorm < 1e-6) {
        return false;
    }
    northB.normalize();

    update(northB, magRefWorld, sigmaMagNoise);
    return true;
}


Eigen::Quaterniond ImuAttitudeFilter::getQuaternion() {
    return qNominal;
}

Eigen::Vector3d ImuAttitudeFilter::getBgNominal() {
    return bgNominal;
}


Eigen::Quaterniond ImuAttitudeFilter::rotvecToQuat(const Eigen::Vector3d& rotvec) {
    const double angle = rotvec.norm();

    if (angle < 1e-8) {
        return Eigen::Quaterniond(1, 0.5 * rotvec.x(), 0.5 * rotvec.y(), 0.5 * rotvec.z()).normalized();
    } else {
        const Eigen::Vector3d axis = rotvec / angle;
        double c = std::cos(0.5 * angle);
        double s = std::sin(0.5 * angle);
        return Eigen::Quaterniond(c, s * axis.x(), s * axis.y(), s * axis.z());
    }
}

Eigen::Matrix3d ImuAttitudeFilter::rotvecToMatrix(const Eigen::Vector3d& rotvec) {
    const double angle = rotvec.norm();

    if (angle < 1e-8) {
        return Eigen::Matrix3d::Identity() + skew(rotvec);
    }

    return Eigen::AngleAxisd(angle, rotvec / angle).toRotationMatrix();
}

Eigen::Matrix3d ImuAttitudeFilter::skew(const Eigen::Vector3d& w) {
    Eigen::Matrix3d S;
    S <<  0.0, -w.z(),  w.y(),
          w.z(),    0.0, -w.x(),
         -w.y(),  w.x(),   0.0;
    return S;
}

Eigen::Matrix3d ImuAttitudeFilter::rightJacobianSO3(const Eigen::Vector3d& theta) {
    const double angle = theta.norm();
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d ThetaX = skew(theta);
    const Eigen::Matrix3d ThetaX2 = ThetaX * ThetaX;

    // small-angle handling
    if (angle < 1e-8) {
        // Taylor expansion:
        // Jr(theta) ≈ I - 1/2 [theta]x + 1/6 [theta]x^2
        return I - 0.5 * ThetaX + (1.0 / 6.0) * ThetaX2;
    }

    const double angle2 = angle * angle;
    const double angle3 = angle2 * angle;

    const double A = (1.0 - std::cos(angle)) / angle2;
    const double B = (angle - std::sin(angle)) / angle3;

    // Jr(theta) ≈ I - (1-cos(angle))/angle^2 [theta]x + (angle - sin(angle))/angle^3 [theta]x^2
    return I - A * ThetaX + B * ThetaX2;
}

