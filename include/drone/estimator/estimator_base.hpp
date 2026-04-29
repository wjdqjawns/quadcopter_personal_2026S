#pragma once
#include "drone/utils.hpp"

namespace drone
{
    class EstimatorBase
    {
    public:
        virtual ~EstimatorBase() = default;
        virtual void update(const IMUData& imu) = 0;
        virtual DroneState getState() const = 0;
        virtual void reset(const DroneState& initial) = 0;
    };
} // namespace drone