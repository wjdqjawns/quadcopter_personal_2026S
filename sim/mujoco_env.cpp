#include "mujoco_env.hpp"
#include <stdexcept>
#include <algorithm>

namespace sim
{
    MujocoEnv::MujocoEnv(const std::string& xml_path)
    {
        // Mesh decoders (OBJ, STL) live in separate plugin .so files in MuJoCo 3.x.
        // They must be loaded before mj_loadXML; calling this repeatedly is safe.
        static bool plugins_loaded = false;
        if (!plugins_loaded)
        {
#ifdef MUJOCO_PLUGIN_DIR
            mj_loadAllPluginLibraries(MUJOCO_PLUGIN_DIR, nullptr);
#endif
            plugins_loaded = true;
        }

        char err[1000] = {};
        m_ = mj_loadXML(xml_path.c_str(), nullptr, err, sizeof(err));
        if (!m_) throw std::runtime_error(std::string("mj_loadXML: ") + err);
        d_ = mj_makeData(m_);
    }

    MujocoEnv::~MujocoEnv()
    {
        mj_deleteData(d_);
        mj_deleteModel(m_);
    }

    void MujocoEnv::step()
    {
        mj_step(m_, d_);
    }

    void MujocoEnv::resetToKeyframe(int key_id)
    {
        mj_resetDataKeyframe(m_, d_, key_id);
        mj_forward(m_, d_);
    }

    void MujocoEnv::setActuators(const drone::Vec4& f)
    {
        for (int i = 0; i < 4; ++i)
        {
            d_->ctrl[i] = std::clamp(f[i], 0.0, 13.0);
        }
    }

    drone::Vec3 MujocoEnv::getPosition() const
    {
        return {d_->qpos[0], d_->qpos[1], d_->qpos[2]};
    }

    drone::Vec3 MujocoEnv::getVelocity() const
    {
        return {d_->qvel[0], d_->qvel[1], d_->qvel[2]};
    }

    drone::Quat MujocoEnv::getOrientation() const
    {
        return drone::Quat(d_->qpos[3], d_->qpos[4], d_->qpos[5], d_->qpos[6]);
    }

    drone::Vec3 MujocoEnv::getAngularVelocity() const
    {
        return {d_->qvel[3], d_->qvel[4], d_->qvel[5]};
    }

    drone::Vec3 MujocoEnv::getSensorGyro() const
    {
        return {d_->sensordata[0], d_->sensordata[1], d_->sensordata[2]};
    }

    drone::Vec3 MujocoEnv::getSensorAccel() const
    {
        return {d_->sensordata[3], d_->sensordata[4], d_->sensordata[5]};
    }

    drone::Quat MujocoEnv::getSensorQuat() const
    {
        return drone::Quat(d_->sensordata[6], d_->sensordata[7], d_->sensordata[8], d_->sensordata[9]);
    }
} // namespace sim