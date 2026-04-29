#pragma once
#include "drone/controller/controller_base.hpp"

namespace drone {

struct PidParams {
    double mass    = 1.325; // kg  (4×0.25 rotors + 0.325 body from x2.xml)
    double gravity = 9.81;
    // Motor geometry — from x2.xml site positions
    double arm_y   = 0.18;   // m  (lateral arm)
    double arm_x   = 0.14;   // m  (longitudinal arm)
    double kq      = 0.0201; // N·m/N (yaw torque coefficient, from gear vector)

    Vec3 kp_pos = {6.0, 6.0, 8.0};
    Vec3 kd_pos = {4.0, 4.0, 5.0};
    Vec3 ki_pos = {0.2, 0.2, 0.4}; // reserved; not yet applied in compute()

    Vec3 kp_att = {8.0, 8.0, 4.0};
    Vec3 kd_att = {2.0, 2.0, 1.0};

    double max_tilt  = 0.5;  // rad
    double max_motor = 13.0; // N (ctrlrange upper bound from x2.xml)
    double min_motor = 0.1;  // N
};

// Maps a ControlInput to four motor force commands [m1..m4] using the Skydio X2
// allocation matrix derived from the gear vectors and site positions in x2.xml.
Vec4 motorMix(const ControlInput& cmd, const PidParams& p);

class PidController : public ControllerBase {
public:
    explicit PidController(const PidParams& p = {});
    ControlInput compute(const DroneState& state,
                         const Vec3& pos_des,
                         const Vec3& vel_des) override;
    void reset() override;

private:
    PidParams p_;
    Vec3      integral_{Vec3::Zero()};
};

} // namespace drone
