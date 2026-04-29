#include "drone/controller/pid.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

static void test_hover()
{
    drone::PidController pid;
    drone::DroneState s;
    s.pos     = {0, 0, 1};
    s.vel     = drone::Vec3::Zero();
    s.quat    = drone::Quat::Identity();
    s.ang_vel = drone::Vec3::Zero();

    drone::ControlInput cmd = pid.compute(s, {0, 0, 1}, drone::Vec3::Zero());

    // At target with no error, thrust should be close to mg.
    double mg = 1.325 * 9.81;
    assert(std::abs(cmd.thrust - mg) < 2.0 && "hover thrust out of range");
    std::cout << "[pass] hover thrust = " << cmd.thrust << " N (expected ~" << mg << ")\n";
}

static void test_motor_mix_hover()
{
    drone::PidParams p;
    drone::ControlInput cmd{p.mass * p.gravity, drone::Vec3::Zero()};
    drone::Vec4 m = drone::motorMix(cmd, p);

    // At hover, all motors should be equal and ≈ mg/4.
    double expected = p.mass * p.gravity / 4.0;
    for (int i = 0; i < 4; ++i)
        assert(std::abs(m[i] - expected) < 0.1 && "motor imbalance at hover");
    std::cout << "[pass] hover motor forces ≈ " << m[0] << " N each\n";
}

static void test_motor_clamp()
{
    drone::PidParams p;
    drone::ControlInput cmd{60.0, drone::Vec3::Zero()}; // unrealistically large
    drone::Vec4 m = drone::motorMix(cmd, p);
    for (int i = 0; i < 4; ++i)
        assert(m[i] <= p.max_motor && "motor force exceeds max");
    std::cout << "[pass] motor clamp enforced\n";
}

static void test_position_error_response()
{
    drone::PidController pid;
    drone::DroneState s;
    s.pos     = {0, 0, 0.5};
    s.vel     = drone::Vec3::Zero();
    s.quat    = drone::Quat::Identity();
    s.ang_vel = drone::Vec3::Zero();

    drone::ControlInput cmd = pid.compute(s, {0, 0, 1.0}, drone::Vec3::Zero());
    assert(cmd.thrust > 1.325 * 9.81 && "no upward acceleration for upward error");
    std::cout << "[pass] upward position error → increased thrust\n";
}

int main()
{
    test_hover();
    test_motor_mix_hover();
    test_motor_clamp();
    test_position_error_response();
    std::cout << "All controller tests passed.\n";
    return 0;
}