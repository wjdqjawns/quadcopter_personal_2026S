#pragma once
#include "drone/controller/controller_base.hpp"
#include <Eigen/Dense>

namespace drone
{
    struct MpcParams
    {
        double mass    = 1.325;
        double gravity = 9.81;
        double arm_y   = 0.18;
        double arm_x   = 0.14;
        double kq      = 0.0201;
        double dt      = 0.01;   // must match mjModel->opt.timestep
        int    N       = 10;     // prediction horizon steps

        // Diagonal state cost [px, py, pz, vx, vy, vz]
        Eigen::Matrix<double,6,1> Q_diag =
            (Eigen::Matrix<double,6,1>() << 10.0, 10.0, 20.0, 4.0, 4.0, 6.0).finished();

        // Diagonal input cost [ax, ay, az]
        Vec3 R_diag = Vec3(0.1, 0.1, 0.1);

        // Attitude inner-loop PD gains (same role as PID inner loop)
        Vec3 kp_att = {8.0, 8.0, 4.0};
        Vec3 kd_att = {2.0, 2.0, 1.0};

        double max_tilt  = 0.5;   // rad
        double max_motor = 13.0;  // N
        double min_motor = 0.1;   // N
    };

    // Motor mixing using MPC geometry params (identical allocation to PID).
    Vec4 mpcMotorMix(const ControlInput& cmd, const MpcParams& p);

    // Unconstrained finite-horizon linear MPC for position / velocity.
    //
    // Outer loop: batch LQR over a linear double-integrator model.
    //   State  x = [pos(3), vel(3)]
    //   Input  u = a_des (world-frame acceleration command, m/s²)
    //
    // Inner loop: PD attitude controller converts a_des to thrust + torques,
    //   identical to the PID inner loop.
    //
    // Gain matrix K_opt is precomputed in the constructor from Q, R, and the
    // horizon N — no per-step matrix inversion is required.
    class MpcController : public ControllerBase
    {
    public:
        explicit MpcController(const MpcParams& p = {});

        ControlInput compute(const DroneState& state,
                            const Vec3& pos_des,
                            const Vec3& vel_des) override;
        void reset() override;

    private:
        MpcParams p_;

        // Batch prediction matrices (precomputed, fixed over the run).
        Eigen::MatrixXd Phi_;    // (6N × 6)  stacked A^k
        Eigen::MatrixXd Theta_;  // (6N × 3N) lower-block-triangular Causal convolution

        // K_opt = (Theta^T Qbar Theta + Rbar)^{-1} Theta^T Qbar  (3N × 6N)
        // Optimal feedback gain applied to the error vector (Xref - Phi*x0).
        Eigen::MatrixXd K_opt_;

        void buildMatrices();
    };
} // namespace drone