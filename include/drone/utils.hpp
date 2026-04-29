#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace drone {

using Vec3 = Eigen::Vector3d;
using Vec4 = Eigen::Vector4d;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;

struct IMUData {
    Vec3   accel; // specific force, body frame, m/s²
    Vec3   gyro;  // angular velocity, body frame, rad/s
    double dt;
};

struct DroneState {
    Vec3 pos;     // world frame, m
    Vec3 vel;     // world frame, m/s
    Quat quat;    // body-in-world (w x y z)
    Vec3 ang_vel; // body frame, rad/s
};

struct ControlInput {
    double thrust; // N, along body z
    Vec3   torque; // N·m, body frame
};

// ZYX Euler angles [roll, pitch, yaw] from body-in-world quaternion.
inline Vec3 quatToEuler(const Quat& q) {
    double sinr = 2.0*(q.w()*q.x() + q.y()*q.z());
    double cosr = 1.0 - 2.0*(q.x()*q.x() + q.y()*q.y());
    double sinp = std::clamp(2.0*(q.w()*q.y() - q.z()*q.x()), -1.0, 1.0);
    double siny = 2.0*(q.w()*q.z() + q.x()*q.y());
    double cosy = 1.0 - 2.0*(q.y()*q.y() + q.z()*q.z());
    return {std::atan2(sinr, cosr), std::asin(sinp), std::atan2(siny, cosy)};
}

// Euler angles [roll, pitch, yaw] → body-in-world quaternion (ZYX).
inline Quat eulerToQuat(double roll, double pitch, double yaw) {
    Quat qr(Eigen::AngleAxisd(roll,  Vec3::UnitX()));
    Quat qp(Eigen::AngleAxisd(pitch, Vec3::UnitY()));
    Quat qy(Eigen::AngleAxisd(yaw,   Vec3::UnitZ()));
    return (qy * qp * qr).normalized();
}

} // namespace drone
