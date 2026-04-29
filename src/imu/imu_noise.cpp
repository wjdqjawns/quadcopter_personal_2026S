#include "drone/imu.hpp"
#include <cmath>

namespace drone {
    ImuNoise::ImuNoise(const ImuNoiseParams& p, unsigned seed): p_(p), rng_(seed)
    {
        std::uniform_real_distribution<double> ud(-1.0, 1.0);
        
        for (int i = 0; i < 3; ++i)
        {
            accel_bias_[i] = p_.accel_bias * ud(rng_);
            gyro_bias_[i]  = p_.gyro_bias  * ud(rng_);
        }
    }

    void ImuNoise::apply(IMUData& data)
    {
        const double inv_sqrt_dt = 1.0 / std::sqrt(std::max(data.dt, 1e-9));
        std::normal_distribution<double> nd;
        for (int i = 0; i < 3; ++i) {
            data.accel[i] += accel_bias_[i] + p_.accel_noise * inv_sqrt_dt * nd(rng_);
            data.gyro[i]  += gyro_bias_[i]  + p_.gyro_noise  * inv_sqrt_dt * nd(rng_);
        }
    }
} // namespace drone