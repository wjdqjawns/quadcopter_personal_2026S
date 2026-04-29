#pragma once
#include "drone/estimator/estimator_base.hpp"

namespace drone {

struct CfParams {
    double alpha = 0.98; // gyro weight (vs accelerometer correction)
};

// Complementary filter for attitude estimation.
// Gyro integration is corrected by accelerometer-derived roll/pitch.
// Yaw drift is not corrected (no magnetometer).
class CfEstimator : public EstimatorBase {
public:
    explicit CfEstimator(const CfParams& p = {});
    void update(const IMUData& imu) override;
    DroneState getState() const override;
    void reset(const DroneState& initial) override;

private:
    CfParams   p_;
    DroneState state_;
};

} // namespace drone
