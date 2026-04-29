#include "drone/estimator/ekf.hpp"

namespace drone
{
    EkfEstimator::EkfEstimator(const EkfParams& p) : p_(p)
    {
        state_.pos     = Vec3::Zero();
        state_.vel     = Vec3::Zero();
        state_.quat    = Quat::Identity();
        state_.ang_vel = Vec3::Zero();
        P_             = Eigen::Matrix<double,6,6>::Identity() * 0.1;
    }

    void EkfEstimator::reset(const DroneState& initial)
    {
        state_ = initial;
        P_     = Eigen::Matrix<double,6,6>::Identity() * 0.1;
    }

    void EkfEstimator::setOrientation(const Quat& q, const Vec3& ang_vel)
    {
        state_.quat    = q;
        state_.ang_vel = ang_vel;
    }

    DroneState EkfEstimator::getState() const { return state_; }

    void EkfEstimator::update(const IMUData& imu)
    {
        // Rotate specific force from body to world, then remove gravity.
        // accel_sensor ≈ a_body − g_body  →  a_world = R·accel_sensor + (0,0,−g)
        Mat3 R        = state_.quat.toRotationMatrix();
        Vec3 a_world  = R * imu.accel + Vec3(0.0, 0.0, -9.81);

        // --- predict ---
        const double dt  = imu.dt;
        const double dt2 = 0.5 * dt * dt;

        Vec3 new_pos = state_.pos + state_.vel * dt + a_world * dt2;
        Vec3 new_vel = state_.vel + a_world * dt;

        // Transition matrix F for [pos, vel]
        Eigen::Matrix<double,6,6> F = Eigen::Matrix<double,6,6>::Identity();
        F.block<3,3>(0,3) = dt * Eigen::Matrix3d::Identity();

        // Process noise
        Eigen::Matrix<double,6,6> Q = Eigen::Matrix<double,6,6>::Zero();
        Q.block<3,3>(0,0) = (p_.proc_noise_pos * dt) * Eigen::Matrix3d::Identity();
        Q.block<3,3>(3,3) = (p_.proc_noise_vel * dt) * Eigen::Matrix3d::Identity();

        P_         = F * P_ * F.transpose() + Q;
        state_.pos = new_pos;
        state_.vel = new_vel;
    }

    void EkfEstimator::fusePosition(const Vec3& pos_meas)
    {
        // H = [I₃ | 0₃] (observe position only)
        Eigen::Matrix<double,3,6> H = Eigen::Matrix<double,3,6>::Zero();
        H.leftCols<3>() = Eigen::Matrix3d::Identity();

        Eigen::Matrix3d           R_meas = p_.meas_noise_pos * Eigen::Matrix3d::Identity();
        Eigen::Matrix3d           S      = H * P_ * H.transpose() + R_meas;
        Eigen::Matrix<double,6,3> K      = P_ * H.transpose() * S.inverse();

        Eigen::Matrix<double,6,1> dx = K * (pos_meas - state_.pos);
        state_.pos += dx.head<3>();
        state_.vel += dx.tail<3>();
        P_ = (Eigen::Matrix<double,6,6>::Identity() - K * H) * P_;
    }
} // namespace drone
