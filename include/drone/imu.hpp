#pragma once
#include "drone/utils.hpp"
#include <random>

namespace drone {

struct ImuNoiseParams {
    double accel_noise = 0.003;  // m/s²/√Hz
    double gyro_noise  = 0.0002; // rad/s/√Hz
    double accel_bias  = 0.01;   // m/s²  (constant offset magnitude)
    double gyro_bias   = 0.001;  // rad/s (constant offset magnitude)
};

class ImuNoise {
public:
    explicit ImuNoise(const ImuNoiseParams& p, unsigned seed = 42);
    void apply(IMUData& data);
private:
    ImuNoiseParams p_;
    std::mt19937   rng_;
    Vec3           accel_bias_{Vec3::Zero()};
    Vec3           gyro_bias_{Vec3::Zero()};
};

// Wraps raw IMU data and applies a noise model.
class Imu {
public:
    explicit Imu(const ImuNoiseParams& p = {});
    IMUData process(const IMUData& raw);
private:
    ImuNoise noise_;
};

} // namespace drone
