#include "drone/estimator/ekf.hpp"
#include "drone/estimator/cf.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

static void test_ekf_init()
{
    drone::EkfEstimator ekf;
    drone::DroneState s = ekf.getState();
    assert(s.pos.norm() < 1e-9 && "EKF initial pos should be zero");
    std::cout << "[pass] EKF initialises to zero\n";
}

static void test_ekf_predict_hover()
{
    drone::EkfEstimator ekf;
    // At hover, accel sensor reads (0,0,9.81) specific force; with R=I
    // world-frame accel = (0,0,9.81) − 9.81 = 0, so pos/vel should not drift.
    drone::IMUData imu{{0, 0, 9.81}, {0, 0, 0}, 0.01};
    for (int i = 0; i < 100; ++i) ekf.update(imu);
    drone::DroneState s = ekf.getState();
    assert(s.pos.norm() < 1e-3 && "EKF should not drift at hover");
    std::cout << "[pass] EKF hover drift < 1 mm over 1 s\n";
}

static void test_ekf_fuse_position()
{
    drone::EkfEstimator ekf;
    // Artificially shift the estimate
    drone::DroneState initial;
    initial.pos = {1.0, 0, 0};
    initial.vel = drone::Vec3::Zero();
    initial.quat = drone::Quat::Identity();
    initial.ang_vel = drone::Vec3::Zero();
    ekf.reset(initial);

    ekf.fusePosition({0, 0, 0}); // fuse true position
    drone::DroneState s = ekf.getState();
    assert(s.pos.norm() < 0.5 && "EKF measurement fusion should reduce pos error");
    std::cout << "[pass] EKF position fusion pulls estimate towards measurement\n";
}

static void test_cf_init()
{
    drone::CfEstimator cf;
    drone::Vec3 euler = drone::quatToEuler(cf.getState().quat);
    assert(euler.norm() < 1e-9 && "CF initial attitude should be zero");
    std::cout << "[pass] CF initialises to zero attitude\n";
}

static void test_cf_accel_correction()
{
    drone::CfEstimator cf;
    // Pure gravity pointing up in body frame → should converge to zero attitude.
    drone::IMUData imu{{0, 0, 9.81}, {0, 0, 0}, 0.01};
    for (int i = 0; i < 200; ++i) cf.update(imu);
    drone::Vec3 euler = drone::quatToEuler(cf.getState().quat);
    assert(std::abs(euler.x()) < 0.01 && std::abs(euler.y()) < 0.01
           && "CF should hold level attitude under gravity");
    std::cout << "[pass] CF holds level attitude under gravity\n";
}

int main()
{
    test_ekf_init();
    test_ekf_predict_hover();
    test_ekf_fuse_position();
    test_cf_init();
    test_cf_accel_correction();
    std::cout << "All estimator tests passed.\n";
    return 0;
}