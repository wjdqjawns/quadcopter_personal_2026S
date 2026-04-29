#pragma once
#include "drone/utils.hpp"

namespace drone {

class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    virtual ControlInput compute(const DroneState& state,
                                 const Vec3& pos_des,
                                 const Vec3& vel_des) = 0;
    virtual void reset() = 0;
};

} // namespace drone
