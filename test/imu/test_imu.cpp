#include "drone/imu.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

static void test_noise_changes_output()
{
    drone::Imu imu;
    drone::IMUData raw{{0, 0, 9.81}, {0, 0, 0}, 0.01};
    drone::IMUData noisy = imu.process(raw);
    // After noise, at least one channel should differ.
    double diff = (noisy.accel - raw.accel).norm() + (noisy.gyro - raw.gyro).norm();
    assert(diff > 1e-12 && "ImuNoise must change the signal");
    std::cout << "[pass] noise changes output  |Δaccel|=" << (noisy.accel-raw.accel).norm() << "\n";
}

static void test_noise_bounded()
{
    drone::ImuNoiseParams p;
    p.accel_noise = 0.01;
    p.gyro_noise  = 0.001;
    p.accel_bias  = 0.0;
    p.gyro_bias   = 0.0;
    drone::Imu imu(p);

    double dt = 0.01;
    double max_accel_dev = 0.0;
    drone::IMUData raw{{0, 0, 9.81}, {0, 0, 0}, dt};
    for (int i = 0; i < 10000; ++i)
    {
        drone::IMUData n = imu.process(raw);
        max_accel_dev = std::max(max_accel_dev, (n.accel - raw.accel).norm());
    }
    // Noise density 0.01 / sqrt(0.01) = 0.1 per sample; 6\sigma \sim 0.6
    assert(max_accel_dev < 2.0 && "accel noise suspiciously large");
    std::cout << "[pass] max accel noise over 10 k steps = " << max_accel_dev << " m/s²\n";
}

static void test_deterministic_seed()
{
    drone::ImuNoiseParams p;
    drone::ImuNoise n1(p, 123), n2(p, 123);
    drone::IMUData d1{{0,0,0},{0,0,0},0.01}, d2 = d1;
    n1.apply(d1);
    n2.apply(d2);
    assert((d1.accel - d2.accel).norm() < 1e-15 && "same seed must give same noise");
    std::cout << "[pass] same seed → deterministic output\n";
}

int main()
{
    test_noise_changes_output();
    test_noise_bounded();
    test_deterministic_seed();
    std::cout << "All IMU tests passed.\n";
    return 0;
}
