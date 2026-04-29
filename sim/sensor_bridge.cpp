#include "sensor_bridge.hpp"

namespace sim
{
    SensorBridge::SensorBridge(const MujocoEnv& env) : env_(env) {}

    drone::DroneState SensorBridge::getGroundTruth() const
    {
        return {env_.getPosition(),
                env_.getVelocity(),
                env_.getOrientation(),
                env_.getAngularVelocity()};
    }

    drone::IMUData SensorBridge::getRawIMU(double dt) const
    {
        // MuJoCo accelerometer = specific force (a − g) in body frame,
        // consistent with a real IMU: reads ≈ (0, 0, 9.81) when hovering level.
        return {env_.getSensorAccel(), env_.getSensorGyro(), dt};
    }
} // namespace sim