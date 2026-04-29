#pragma once
#include "mujoco_env.hpp"
#include "drone/utils.hpp"
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <deque>

namespace sim {

// All per-frame data the visualizer needs to paint state overlays.
struct SimDisplay {
    double              time{0};
    drone::DroneState   ground_truth;   // from MuJoCo qpos/qvel
    drone::DroneState   estimated;      // from EKF + CF
    drone::ControlInput cmd;
    drone::Vec4         motors{drone::Vec4::Zero()};
    drone::Vec3         target_pos{drone::Vec3::Zero()};
};

// History buffer for plotting
struct PlotHistory {
    std::deque<float> time_vals;
    std::deque<float> pos_x, pos_y, pos_z;
    std::deque<float> vel_x, vel_y, vel_z;
    std::deque<float> roll, pitch, yaw;
    std::deque<float> motor1, motor2, motor3, motor4;
    std::deque<float> thrust;
    
    static constexpr int MAX_HISTORY = 1000; // ~10 seconds at 100 Hz
    
    void push(const SimDisplay& d) {
        drone::Vec3 eul = drone::quatToEuler(d.ground_truth.quat);
        
        time_vals.push_back(d.time);
        pos_x.push_back(d.ground_truth.pos.x());
        pos_y.push_back(d.ground_truth.pos.y());
        pos_z.push_back(d.ground_truth.pos.z());
        vel_x.push_back(d.ground_truth.vel.x());
        vel_y.push_back(d.ground_truth.vel.y());
        vel_z.push_back(d.ground_truth.vel.z());
        roll.push_back(eul.x() * 180.0 / M_PI);
        pitch.push_back(eul.y() * 180.0 / M_PI);
        yaw.push_back(eul.z() * 180.0 / M_PI);
        motor1.push_back(d.motors[0]);
        motor2.push_back(d.motors[1]);
        motor3.push_back(d.motors[2]);
        motor4.push_back(d.motors[3]);
        thrust.push_back(d.cmd.thrust);
        
        // Keep history size bounded
        while (time_vals.size() > MAX_HISTORY) {
            time_vals.pop_front();
            pos_x.pop_front(); pos_y.pop_front(); pos_z.pop_front();
            vel_x.pop_front(); vel_y.pop_front(); vel_z.pop_front();
            roll.pop_front(); pitch.pop_front(); yaw.pop_front();
            motor1.pop_front(); motor2.pop_front(); 
            motor3.pop_front(); motor4.pop_front();
            thrust.pop_front();
        }
    }
};

class Visualizer {
public:
    explicit Visualizer(MujocoEnv& env, int width = 1280, int height = 720);
    ~Visualizer();

    Visualizer(const Visualizer&)            = delete;
    Visualizer& operator=(const Visualizer&) = delete;

    bool shouldClose() const;
    void render(const SimDisplay& d);

private:
    MujocoEnv&  env_;
    GLFWwindow* window_{nullptr};
    mjvCamera   cam_{};
    mjvOption   opt_{};
    mjvScene    scn_{};
    mjrContext  con_{};

    bool   btn_left_{false}, btn_right_{false};
    double last_x_{0}, last_y_{0};
    
    // History tracking for plots
    PlotHistory history_;

    // Singleton pointer used by static GLFW callbacks.
    static Visualizer* instance_;

    static void keyCb(GLFWwindow*, int key, int, int action, int);
    static void scrollCb(GLFWwindow*, double, double yoffset);
    static void mouseBtnCb(GLFWwindow*, int button, int action, int);
    static void cursorCb(GLFWwindow*, double xpos, double ypos);
    
    // Rendering helpers
    void renderTextOverlays(const SimDisplay& d, int W, int H);
    void drawSimplePlot(int x, int y, int w, int h, 
                       const std::deque<float>& data, 
                       float min_val, float max_val, const char* label);
};

} // namespace sim

