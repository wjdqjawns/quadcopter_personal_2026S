#include "drone/imu.hpp"

namespace drone
{
    Imu::Imu(const ImuNoiseParams& p) : noise_(p) {}

    IMUData Imu::process(const IMUData& raw)
    {
        IMUData out = raw;
        noise_.apply(out);
        return out;
    }
} // namespace drone