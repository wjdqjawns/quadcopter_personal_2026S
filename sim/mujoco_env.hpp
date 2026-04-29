#pragma once
#include "drone/utils.hpp"
#include <mujoco/mujoco.h>
#include <string>

namespace sim
{
    class MujocoEnv
    {
    public:
        explicit MujocoEnv(const std::string& xml_path);
        ~MujocoEnv();

        MujocoEnv(const MujocoEnv&)            = delete;
        MujocoEnv& operator=(const MujocoEnv&) = delete;

        void step();
        void resetToKeyframe(int key_id = 0);

        // Clamps each value to [0, 13] before writing ctrl[].
        void setActuators(const drone::Vec4& motor_forces);

        // Ground-truth state from qpos / qvel.
        drone::Vec3 getPosition()        const;
        drone::Vec3 getVelocity()        const;
        drone::Quat getOrientation()     const;
        drone::Vec3 getAngularVelocity() const;

        // Raw sensor readings (sensor order in x2.xml: gyro, accel, framequat).
        drone::Vec3 getSensorGyro()  const; // sensordata[0:3]
        drone::Vec3 getSensorAccel() const; // sensordata[3:6]
        drone::Quat getSensorQuat()  const; // sensordata[6:10]

        mjModel* model() const { return m_; }
        mjData*  data()  const { return d_; }
        double   time()  const { return d_->time; }

    private:
        mjModel* m_{nullptr};
        mjData*  d_{nullptr};
    };
} // namespace sim
