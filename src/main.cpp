#include "drone/controller/pid.hpp"
#include "drone/estimator/ekf.hpp"
#include "drone/estimator/cf.hpp"
#include "drone/trajectory.hpp"
#include "drone/imu.hpp"
#include "mujoco_env.hpp"
#include "sensor_bridge.hpp"
#include "visualizer.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

// ---------------------------------------------------------------------------
// Config helpers
// ---------------------------------------------------------------------------
static drone::Vec3 yamlVec3(const YAML::Node& n, drone::Vec3 def = drone::Vec3::Zero())
{
    if (!n || !n.IsSequence() || n.size() < 3) return def;
    return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
}

static drone::PidParams loadPid(const std::string& path)
{
    drone::PidParams p;
    try
    {
        YAML::Node y  = YAML::LoadFile(path)["pid"];
        p.mass        = y["mass"].as<double>(p.mass);
        p.gravity     = y["gravity"].as<double>(p.gravity);
        p.arm_y       = y["arm_y"].as<double>(p.arm_y);
        p.arm_x       = y["arm_x"].as<double>(p.arm_x);
        p.kq          = y["kq"].as<double>(p.kq);
        p.kp_pos      = yamlVec3(y["pos"]["kp"], p.kp_pos);
        p.kd_pos      = yamlVec3(y["pos"]["kd"], p.kd_pos);
        p.kp_att      = yamlVec3(y["att"]["kp"], p.kp_att);
        p.kd_att      = yamlVec3(y["att"]["kd"], p.kd_att);
        p.max_tilt    = y["max_tilt"].as<double>(p.max_tilt);
        p.max_motor   = y["max_motor"].as<double>(p.max_motor);
        p.min_motor   = y["min_motor"].as<double>(p.min_motor);
    }
    catch (...)
    {
        std::cerr << "[warn] Could not load " << path << ", using defaults\n";
    }
    return p;
}

static drone::EkfParams loadEkf(const std::string& path)
{
    drone::EkfParams p;
    try
    {
        YAML::Node y      = YAML::LoadFile(path)["ekf"];
        p.proc_noise_pos  = y["proc_noise_pos"].as<double>(p.proc_noise_pos);
        p.proc_noise_vel  = y["proc_noise_vel"].as<double>(p.proc_noise_vel);
        p.meas_noise_pos  = y["meas_noise_pos"].as<double>(p.meas_noise_pos);
    }
    catch (...)
    {
        std::cerr << "[warn] Could not load EKF params from " << path << "\n";
    }
    return p;
}

static drone::CfParams loadCf(const std::string& path)
{
    drone::CfParams p;
    try
    {
        p.alpha = YAML::LoadFile(path)["complementary_filter"]["alpha"].as<double>(p.alpha);
    }
    catch (...)
    {
    }
    return p;
}

static drone::ImuNoiseParams loadImu(const std::string& path)
{
    drone::ImuNoiseParams p;
    try
    {
        YAML::Node y   = YAML::LoadFile(path)["imu"];
        p.accel_noise  = y["accel_noise"].as<double>(p.accel_noise);
        p.gyro_noise   = y["gyro_noise"].as<double>(p.gyro_noise);
        p.accel_bias   = y["accel_bias"].as<double>(p.accel_bias);
        p.gyro_bias    = y["gyro_bias"].as<double>(p.gyro_bias);
    }
    catch (...)
    {

    }
    return p;
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv)
// ---------------------------------------------------------------------------
{
    // Allow overriding the sim config path via CLI.
    std::string sim_cfg = "config/sim/skydio_x2.yaml";
    if (argc > 1) sim_cfg = argv[1];

    YAML::Node sim;
    try
    {
        sim = YAML::LoadFile(sim_cfg)["simulation"];
    }
    catch (const std::exception& e)
    {
        std::cerr << "Cannot load " << sim_cfg << ": " << e.what() << "\n";
        return 1;
    }

    const std::string model_path = sim["model_path"].as<std::string>("asset/skydio_x2/scene.xml");
    const std::string log_path   = sim["log_path"].as<std::string>("data/logs/sim_log.csv");
    const double      duration   = sim["duration"].as<double>(30.0);
    const bool        visualize  = sim["visualize"].as<bool>(true);

    // --- sim objects ---
    sim::MujocoEnv    env(model_path);
    sim::SensorBridge bridge(env);

    const double dt = env.model()->opt.timestep; // 0.01 s from x2.xml

    // --- load params ---
    drone::PidParams       pid_p = loadPid("config/controller_params.yaml");
    drone::EkfParams       ekf_p = loadEkf("config/estimator_params.yaml");
    drone::CfParams        cf_p  = loadCf("config/estimator_params.yaml");
    drone::ImuNoiseParams  imu_p = loadImu("config/imu_params.yaml");

    // --- create algorithm objects ---
    drone::PidController pid(pid_p);
    drone::EkfEstimator  ekf(ekf_p);
    drone::CfEstimator   cf(cf_p);
    drone::Imu           imu_model(imu_p);

    // --- trajectory: takeoff → 1 m hover → circle → land ---
    drone::Vec3 home(0.0, 0.0, 0.3);
    drone::Vec3 cruise(0.0, 0.0, 1.0);

    std::vector<drone::Waypoint> wps = {
        {home,   drone::Vec3::Zero(),  0.0},
        {cruise, drone::Vec3::Zero(),  5.0},
        {cruise, drone::Vec3::Zero(), 10.0},
    };
    // Append a circle (1 m radius, 8 s period, 15 s long)
    auto circ = drone::Trajectory::circle(cruise, 1.0, 0.0, 8.0, 15.0);
    for (int i = 0; i <= 48; ++i) {
        double t = 15.0 * i / 48;
        wps.push_back({circ.getPosition(t), circ.getVelocity(t), 10.0 + t});
    }
    // Return home
    wps.push_back({home, drone::Vec3::Zero(), duration});
    drone::Trajectory traj(wps);

    // --- optional visualizer ---
    std::unique_ptr<sim::Visualizer> vis;
    if (visualize) {
        try { vis = std::make_unique<sim::Visualizer>(env); }
        catch (const std::exception& e) {
            std::cerr << "[warn] Visualizer disabled: " << e.what() << "\n";
        }
    }

    // --- log file ---
    std::ofstream log(log_path);
    if (!log) std::cerr << "[warn] Cannot open log: " << log_path << "\n";
    else log << "t,px,py,pz,vx,vy,vz,roll,pitch,yaw,"
                "thrust,tau_x,tau_y,tau_z,m1,m2,m3,m4,"
                "est_px,est_py,est_pz,est_vx,est_vy,est_vz\n";

    // --- reset to hover keyframe ---
    env.resetToKeyframe(0); // "hover" key from x2.xml
    drone::DroneState gt0 = bridge.getGroundTruth();
    ekf.reset(gt0);
    cf.reset(gt0);

    std::cout << "Simulation started  duration=" << duration
              << "s  dt=" << dt << "s\n";

    // -----------------------------------------------------------------------
    // Main loop
    // -----------------------------------------------------------------------
    while (env.time() < duration) {
        if (vis && vis->shouldClose()) break;

        // 1. Sense
        drone::IMUData raw   = bridge.getRawIMU(dt);
        drone::IMUData noisy = imu_model.process(raw);

        // 2. Estimate attitude (CF) → inject into EKF → estimate pos/vel
        cf.update(noisy);
        drone::DroneState cf_state = cf.getState();
        ekf.setOrientation(cf_state.quat, cf_state.ang_vel);
        ekf.update(noisy);

        drone::DroneState est = ekf.getState();
        est.quat    = cf_state.quat;
        est.ang_vel = cf_state.ang_vel;

        // 3. Control
        double t = env.time();
        drone::ControlInput cmd = pid.compute(est,
                                              traj.getPosition(t),
                                              traj.getVelocity(t));
        drone::Vec4 motors = drone::motorMix(cmd, pid_p);

        // 4. Apply & step
        env.setActuators(motors);
        env.step();

        // 5. Log + render (one shared ground-truth read per step)
        drone::DroneState gt = bridge.getGroundTruth();
        if (log) {
            drone::Vec3       eul = drone::quatToEuler(gt.quat);
            log << t          << ',' << gt.pos.x()  << ',' << gt.pos.y()  << ',' << gt.pos.z()
                              << ',' << gt.vel.x()  << ',' << gt.vel.y()  << ',' << gt.vel.z()
                              << ',' << eul.x()     << ',' << eul.y()     << ',' << eul.z()
                              << ',' << cmd.thrust  << ',' << cmd.torque.x() << ','
                                                              << cmd.torque.y() << ','
                                                              << cmd.torque.z()
                              << ',' << motors[0]   << ',' << motors[1]   << ','
                                     << motors[2]   << ',' << motors[3]
                              << ',' << est.pos.x() << ',' << est.pos.y() << ',' << est.pos.z()
                              << ',' << est.vel.x() << ',' << est.vel.y() << ',' << est.vel.z()
                              << '\n';
        }

        // 6. Render
        if (vis) {
            sim::SimDisplay disp;
            disp.time         = t;
            disp.ground_truth = gt;
            disp.estimated    = est;
            disp.cmd          = cmd;
            disp.motors       = motors;
            disp.target_pos   = traj.getPosition(t);
            vis->render(disp);
        }
    }

    std::cout << "Simulation finished  t=" << env.time() << "s\n";
    if (log) std::cout << "Log saved to " << log_path << "\n";
    return 0;
}
