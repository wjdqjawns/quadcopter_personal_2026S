#include "drone/estimator/cf.hpp"
#include <cmath>
#include <algorithm>

namespace drone
{
    CfEstimator::CfEstimator(const CfParams& p) : p_(p)
    {
        state_.pos     = Vec3::Zero();
        state_.vel     = Vec3::Zero();
        state_.quat    = Quat::Identity();
        state_.ang_vel = Vec3::Zero();
    }

    void CfEstimator::reset(const DroneState& initial)
    {
        state_ = initial;
    }

    DroneState CfEstimator::getState() const { return state_; }

    void CfEstimator::update(const IMUData& imu)
    {
        Vec3   euler = quatToEuler(state_.quat);
        double phi   = euler.x(); // roll
        double theta = euler.y(); // pitch
        double psi   = euler.z(); // yaw

        // Gyro → Euler-angle rate kinematics
        double cp = std::cos(phi), sp = std::sin(phi);
        double ct = std::cos(theta), tt = std::tan(theta);
        double ct_safe = std::max(std::abs(ct), 0.05) * (ct < 0 ? -1.0 : 1.0);

        double dphi   = imu.gyro.x() + imu.gyro.y()*sp*tt + imu.gyro.z()*cp*tt;
        double dtheta = imu.gyro.y()*cp                    - imu.gyro.z()*sp;
        double dpsi   = imu.gyro.y()*sp/ct_safe            + imu.gyro.z()*cp/ct_safe;

        phi   += dphi   * imu.dt;
        theta += dtheta * imu.dt;
        psi   += dpsi   * imu.dt;

        // Accelerometer correction for roll + pitch (valid when |a| ≈ g).
        double an = imu.accel.norm();
        if (an > 5.0 && an < 20.0)
        {
            double phi_acc   = std::atan2(-imu.accel.y(), imu.accel.z());
            double pitch_acc = std::atan2(imu.accel.x(), std::sqrt(imu.accel.y()*imu.accel.y() + imu.accel.z()*imu.accel.z()));
            phi   = p_.alpha * phi   + (1.0 - p_.alpha) * phi_acc;
            theta = p_.alpha * theta + (1.0 - p_.alpha) * pitch_acc;
        }

        state_.quat    = eulerToQuat(phi, theta, psi);
        state_.ang_vel = imu.gyro;
    }
} // namespace drone
