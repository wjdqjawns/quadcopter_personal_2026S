#pragma once

#include "drone/estimator/estimator_base.hpp"
#include <Eigen/Dense>

namespace drone
{
    struct EkfParams
    {
        double proc_noise_pos = 0.01; // m²/s³
        double proc_noise_vel = 0.1;  // m²/s³
        double meas_noise_pos = 0.05; // m²
    };

    // 6-state linear Kalman filter tracking position + velocity.
    // Orientation is injected externally (e.g. from CfEstimator) so the EKF
    // can rotate IMU specific force into the world frame.
    // Call fusePosition() whenever a position measurement (GPS, mocap…) arrives.
    class EkfEstimator : public EstimatorBase
    {
    public:
        explicit EkfEstimator(const EkfParams& p = {});

        void update(const IMUData& imu) override;
        DroneState getState() const override;
        void reset(const DroneState& initial) override;

        // Inject orientation from an attitude estimator before calling update().
        void setOrientation(const Quat& q, const Vec3& ang_vel);

        // Fuse an external position measurement (optional).
        void fusePosition(const Vec3& pos_meas);

    private:
        EkfParams p_;
        DroneState state_;
        Eigen::Matrix<double,6,6> P_; // state covariance
    };
} // namespace drone