#pragma once
#include "drone/utils.hpp"
#include "mujoco_env.hpp"

namespace sim
{
    class SensorBridge
    {
    public:
        explicit SensorBridge(const MujocoEnv& env);

        // Ground-truth state from qpos/qvel (use for logging, not for control).
        drone::DroneState getGroundTruth() const;

        // Raw IMU reading from MuJoCo sensor data (physics-accurate, no extra noise).
        drone::IMUData getRawIMU(double dt) const;

    private:
        const MujocoEnv& env_;
    };
} // namespace sim