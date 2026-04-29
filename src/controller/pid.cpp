#include "drone/controller/pid.hpp"
#include <cmath>
#include <algorithm>

namespace drone
{
    // Motor allocation inverse for the Skydio X2 X-configuration.
    //
    // Site positions from x2.xml (body frame, x=forward, y=left, z=up):
    //   m1 (thrust1): r=(-0.14, -0.18)  gear-yaw = -0.0201
    //   m2 (thrust2): r=(-0.14,  0.18)  gear-yaw = +0.0201
    //   m3 (thrust3): r=( 0.14,  0.18)  gear-yaw = -0.0201
    //   m4 (thrust4): r=( 0.14, -0.18)  gear-yaw = +0.0201
    //
    // Allocation matrix A maps [m1..m4] → [F, τx, τy, τz].
    // A^{-1} = (1/4) · [[1, -1/ly,  1/lx, -1/kq],
    //                    [1,  1/ly,  1/lx,  1/kq],
    //                    [1,  1/ly, -1/lx, -1/kq],
    //                    [1, -1/ly, -1/lx,  1/kq]]
    Vec4 motorMix(const ControlInput& cmd, const PidParams& p) {
        const double ily = 1.0 / p.arm_y;
        const double ilx = 1.0 / p.arm_x;
        const double ikq = 1.0 / p.kq;
        const double F  = cmd.thrust;
        const double tx = cmd.torque.x();
        const double ty = cmd.torque.y();
        const double tz = cmd.torque.z();

        Vec4 m;
        m[0] = 0.25 * (F - ily*tx + ilx*ty - ikq*tz);
        m[1] = 0.25 * (F + ily*tx + ilx*ty + ikq*tz);
        m[2] = 0.25 * (F + ily*tx - ilx*ty - ikq*tz);
        m[3] = 0.25 * (F - ily*tx - ilx*ty + ikq*tz);
        return m.cwiseMax(p.min_motor).cwiseMin(p.max_motor);
    }

    PidController::PidController(const PidParams& p) : p_(p) {}

    void PidController::reset() { integral_ = Vec3::Zero(); }

    ControlInput PidController::compute(const DroneState& state, const Vec3& pos_des, const Vec3& vel_des)
    {
        const double g = p_.gravity;
        const double m = p_.mass;

        // --- position P+D loop → desired acceleration in world frame ---
        Vec3 pos_err = pos_des - state.pos;
        Vec3 vel_err = vel_des - state.vel;
        Vec3 a_des   = p_.kp_pos.cwiseProduct(pos_err)
                    + p_.kd_pos.cwiseProduct(vel_err);
        a_des.z() = std::clamp(a_des.z(), -0.9*g, 2.0*g);

        // --- total thrust from desired vertical + horizontal acceleration ---
        Vec3   n(a_des.x(), a_des.y(), a_des.z() + g);
        double n_norm = std::max(n.norm(), 0.01);
        double thrust = std::clamp(m * n_norm, 0.5*m*g, 4.0*m*g);

        // --- attitude setpoint: body z-axis must align with n_hat ---
        Vec3   n_hat     = n / n_norm;
        double sin_tilt  = std::sin(p_.max_tilt);
        double pitch_des = std::asin(std::clamp(n_hat.x(), -sin_tilt, sin_tilt));
        double cp        = std::cos(pitch_des);
        double roll_des  = (std::abs(cp) > 0.01) ? -std::asin(std::clamp(n_hat.y() / cp, -sin_tilt, sin_tilt)) : 0.0;

        // --- attitude P+D loop → torques ---
        Vec3 euler   = quatToEuler(state.quat);
        Vec3 att_err = Vec3(roll_des - euler.x(), pitch_des - euler.y(), -euler.z()); // hold yaw = 0
        att_err.z()  = std::remainder(att_err.z(), 2.0 * M_PI);

        Vec3 torque = p_.kp_att.cwiseProduct(att_err) - p_.kd_att.cwiseProduct(state.ang_vel);

        return {thrust, torque};
    }
} // namespace drone
