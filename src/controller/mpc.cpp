#include "drone/controller/mpc.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

namespace drone {

// ---------------------------------------------------------------------------
// Motor allocation (identical formula to PID, parameterised by MpcParams).
// ---------------------------------------------------------------------------
Vec4 mpcMotorMix(const ControlInput& cmd, const MpcParams& p)
{
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

// ---------------------------------------------------------------------------
MpcController::MpcController(const MpcParams& p) : p_(p)
{
    buildMatrices();
}

void MpcController::reset() {}

// ---------------------------------------------------------------------------
// Precompute batch matrices and the closed-form optimal gain K_opt.
//
// Discrete linear model  x_{k+1} = A x_k + B u_k  with
//   A = [I₃  dt·I₃]    B = [0.5·dt²·I₃]
//       [0₃    I₃  ]        [dt·I₃      ]
//
// Batch form over N steps:
//   X = Phi · x₀ + Theta · U
//   Phi  (6N×6)  : row-block i = A^(i+1)
//   Theta(6N×3N) : block(i,j) = A^(i-j)·B  for i≥j, 0 otherwise
//
// Unconstrained QP solution:
//   U* = K_opt · (X_ref − Phi·x₀)
//   K_opt = (Θᵀ Q̄ Θ + R̄)⁻¹ Θᵀ Q̄           (3N × 6N)
// ---------------------------------------------------------------------------
void MpcController::buildMatrices()
{
    constexpr int n = 6;  // state dimension
    constexpr int m = 3;  // input dimension
    const int     N = p_.N;
    const double  dt = p_.dt;

    // Discrete A, B
    Eigen::Matrix<double,6,6> A = Eigen::Matrix<double,6,6>::Identity();
    A.block<3,3>(0,3) = dt * Eigen::Matrix3d::Identity();

    Eigen::Matrix<double,6,3> B = Eigen::Matrix<double,6,3>::Zero();
    B.block<3,3>(0,0) = (0.5 * dt * dt) * Eigen::Matrix3d::Identity();
    B.block<3,3>(3,0) = dt             * Eigen::Matrix3d::Identity();

    // Powers of A:  Apow[k] = A^k,  k = 0 … N
    std::vector<Eigen::Matrix<double,6,6>> Apow(N + 1);
    Apow[0] = Eigen::Matrix<double,6,6>::Identity();
    for (int k = 1; k <= N; ++k)
        Apow[k] = Apow[k-1] * A;

    // Phi (6N × 6)
    Phi_.resize(N*n, n);
    for (int i = 0; i < N; ++i)
        Phi_.block(i*n, 0, n, n) = Apow[i+1];

    // Theta (6N × 3N)
    Theta_.resize(N*n, N*m);
    Theta_.setZero();
    for (int i = 0; i < N; ++i)
        for (int j = 0; j <= i; ++j)
            Theta_.block(i*n, j*m, n, m) = Apow[i-j] * B;

    // Block-diagonal cost matrices
    Eigen::MatrixXd Qbar = Eigen::MatrixXd::Zero(N*n, N*n);
    Eigen::MatrixXd Rbar = Eigen::MatrixXd::Zero(N*m, N*m);

    Eigen::Matrix<double,6,6> Q = p_.Q_diag.asDiagonal();
    Eigen::Matrix3d            R = p_.R_diag.asDiagonal();

    for (int i = 0; i < N; ++i) {
        Qbar.block(i*n, i*n, n, n) = Q;
        Rbar.block(i*m, i*m, m, m) = R;
    }

    // K_opt = (Θᵀ Q̄ Θ + R̄)⁻¹ Θᵀ Q̄
    Eigen::MatrixXd TtQ = Theta_.transpose() * Qbar;  // (3N × 6N)
    Eigen::MatrixXd H   = TtQ * Theta_ + Rbar;        // (3N × 3N)
    K_opt_ = H.ldlt().solve(TtQ);                     // (3N × 6N)
}

// ---------------------------------------------------------------------------
// Per-step control computation.
// ---------------------------------------------------------------------------
ControlInput MpcController::compute(const DroneState& state,
                                     const Vec3& pos_des,
                                     const Vec3& vel_des)
{
    constexpr int n = 6;
    const int     N = p_.N;
    const double  g = p_.gravity;
    const double  mass = p_.mass;

    // Current state vector
    Eigen::Matrix<double,6,1> x0;
    x0 << state.pos, state.vel;

    // Constant reference over the horizon
    Eigen::Matrix<double,6,1> xref;
    xref << pos_des, vel_des;

    Eigen::VectorXd Xref(N * n);
    for (int i = 0; i < N; ++i)
        Xref.segment(i*n, n) = xref;

    // Optimal control sequence; extract first input u₀*
    Eigen::VectorXd Uopt = K_opt_ * (Xref - Phi_ * x0);
    Vec3 a_des = Uopt.head<3>();

    // ---- Convert desired acceleration to thrust + attitude torques ----
    // (same geometry as PID outer→inner cascade)
    a_des.z() = std::clamp(a_des.z(), -0.9*g, 2.0*g);

    Vec3   thrust_vec(a_des.x(), a_des.y(), a_des.z() + g);
    double tv_norm = std::max(thrust_vec.norm(), 0.01);
    double thrust  = std::clamp(mass * tv_norm, 0.5*mass*g, 4.0*mass*g);

    Vec3   n_hat     = thrust_vec / tv_norm;
    double sin_tilt  = std::sin(p_.max_tilt);
    double pitch_des = std::asin(std::clamp(n_hat.x(), -sin_tilt, sin_tilt));
    double cp        = std::cos(pitch_des);
    double roll_des  = (std::abs(cp) > 0.01)
                     ? -std::asin(std::clamp(n_hat.y() / cp, -sin_tilt, sin_tilt))
                     : 0.0;

    Vec3 euler   = quatToEuler(state.quat);
    Vec3 att_err = Vec3(roll_des - euler.x(), pitch_des - euler.y(), -euler.z());
    att_err.z()  = std::remainder(att_err.z(), 2.0 * M_PI);

    Vec3 torque = p_.kp_att.cwiseProduct(att_err)
                - p_.kd_att.cwiseProduct(state.ang_vel);

    return {thrust, torque};
}

} // namespace drone
