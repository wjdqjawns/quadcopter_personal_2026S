#include "visualizer.hpp"
#include <stdexcept>
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <iostream>

namespace sim {

Visualizer* Visualizer::instance_ = nullptr;

Visualizer::Visualizer(MujocoEnv& env, int width, int height) : env_(env) {
    instance_ = this;

    if (!glfwInit())
        throw std::runtime_error("glfwInit failed");

    glfwWindowHint(GLFW_SAMPLES, 4);
    window_ = glfwCreateWindow(width, height, "Drone Sim — Skydio X2", nullptr, nullptr);
    if (!window_) { glfwTerminate(); throw std::runtime_error("glfwCreateWindow failed"); }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    glfwSetKeyCallback(window_,         keyCb);
    glfwSetScrollCallback(window_,      scrollCb);
    glfwSetMouseButtonCallback(window_, mouseBtnCb);
    glfwSetCursorPosCallback(window_,   cursorCb);

    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);

    mjv_makeScene(env_.model(), &scn_, 2000);
    mjr_makeContext(env_.model(), &con_, mjFONTSCALE_150);

    // Track the drone body automatically.
    int body_id = mj_name2id(env_.model(), mjOBJ_BODY, "x2");
    cam_.type       = mjCAMERA_TRACKING;
    cam_.trackbodyid = body_id;
    cam_.distance   = 2.0;
    cam_.elevation  = -20.0;
    cam_.azimuth    = 45.0;
}

Visualizer::~Visualizer() {
    mjv_freeScene(&scn_);
    mjr_freeContext(&con_);
    if (window_) glfwDestroyWindow(window_);
    glfwTerminate();
    instance_ = nullptr;
}

bool Visualizer::shouldClose() const {
    return glfwWindowShouldClose(window_);
}

void Visualizer::renderTextOverlays(const SimDisplay& d, int W, int H) {
    mjrRect vp{0, 0, W, H};
    
    constexpr double R2D = 180.0 / M_PI;
    drone::Vec3 eul = drone::quatToEuler(d.ground_truth.quat);
    drone::Vec3 err = d.target_pos - d.ground_truth.pos;

    // Top-left: ground-truth pose ─────────────────────────────────────────────
    char tl_title[512], tl_val[512];
    
    // Calculate min/max from history for ground truth position
    float pos_x_min = 999, pos_x_max = -999;
    float pos_z_min = 999, pos_z_max = -999;
    for (float v : history_.pos_x) { pos_x_min = std::min(pos_x_min, v); pos_x_max = std::max(pos_x_max, v); }
    for (float v : history_.pos_z) { pos_z_min = std::min(pos_z_min, v); pos_z_max = std::max(pos_z_max, v); }
    
    std::snprintf(tl_title, sizeof(tl_title),
        "Time (s)\n"
        "\n"
        "Pos X (m)\n"
        "Pos Y (m)\n"
        "Pos Z (m)\n"
        "\n"
        "Vel X (m/s)\n"
        "Vel Y (m/s)\n"
        "Vel Z (m/s)\n"
        "\n"
        "Roll  (deg)\n"
        "Pitch (deg)\n"
        "Yaw   (deg)");
    std::snprintf(tl_val, sizeof(tl_val),
        "%.2f\n"
        "\n"
        "%+8.4f [%+.2f]\n" "%+8.4f\n" "%+8.4f [%+.2f]\n"
        "\n"
        "%+8.4f\n" "%+8.4f\n" "%+8.4f\n"
        "\n"
        "%+8.3f\n" "%+8.3f\n" "%+8.3f",
        d.time,
        d.ground_truth.pos.x(), pos_x_min, d.ground_truth.pos.y(), 
        d.ground_truth.pos.z(), pos_z_min,
        d.ground_truth.vel.x(), d.ground_truth.vel.y(), d.ground_truth.vel.z(),
        eul.x()*R2D, eul.y()*R2D, eul.z()*R2D);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, vp, tl_title, tl_val, &con_);

    // Bottom-left: target + tracking error ────────────────────────────────────
    char bl_title[256], bl_val[256];
    std::snprintf(bl_title, sizeof(bl_title),
        "Target X (m)\n"
        "Target Y (m)\n"
        "Target Z (m)\n"
        "\n"
        "Err X (m)\n"
        "Err Y (m)\n"
        "Err Z (m)");
    std::snprintf(bl_val, sizeof(bl_val),
        "%+8.4f\n" "%+8.4f\n" "%+8.4f\n"
        "\n"
        "%+8.4f\n" "%+8.4f\n" "%+8.4f",
        d.target_pos.x(), d.target_pos.y(), d.target_pos.z(),
        err.x(), err.y(), err.z());
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, vp, bl_title, bl_val, &con_);

    // Top-right: EKF estimate ─────────────────────────────────────────────────
    char tr_title[256], tr_val[256];
    drone::Vec3 eul_est = drone::quatToEuler(d.estimated.quat);
    std::snprintf(tr_title, sizeof(tr_title),
        "EKF Pos X (m)\n"
        "EKF Pos Y (m)\n"
        "EKF Pos Z (m)\n"
        "\n"
        "EKF Vel X\n"
        "EKF Vel Y\n"
        "EKF Vel Z\n"
        "\n"
        "CF Roll  (deg)\n"
        "CF Pitch (deg)\n"
        "CF Yaw   (deg)");
    std::snprintf(tr_val, sizeof(tr_val),
        "%+8.4f\n" "%+8.4f\n" "%+8.4f\n"
        "\n"
        "%+8.4f\n" "%+8.4f\n" "%+8.4f\n"
        "\n"
        "%+8.3f\n" "%+8.3f\n" "%+8.3f",
        d.estimated.pos.x(), d.estimated.pos.y(), d.estimated.pos.z(),
        d.estimated.vel.x(), d.estimated.vel.y(), d.estimated.vel.z(),
        eul_est.x()*R2D, eul_est.y()*R2D, eul_est.z()*R2D);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, vp, tr_title, tr_val, &con_);

    // Bottom-right: control outputs + history stats ─────────────────────────
    float motor_min = 999, motor_max = -999;
    for (float v : history_.motor1) { motor_min = std::min(motor_min, v); motor_max = std::max(motor_max, v); }
    
    char br_title[256], br_val[256];
    std::snprintf(br_title, sizeof(br_title),
        "Thrust (N)\n"
        "Tau X (N m)\n"
        "Tau Y (N m)\n"
        "Tau Z (N m)\n"
        "\n"
        "Motor 1 (N)\n"
        "Motor 2 (N)\n"
        "Motor 3 (N)\n"
        "Motor 4 (N)\n"
        "\n"
        "Motors[min-max]");
    std::snprintf(br_val, sizeof(br_val),
        "%8.4f\n"
        "%+8.4f\n" "%+8.4f\n" "%+8.4f\n"
        "\n"
        "%8.4f\n" "%8.4f\n" "%8.4f\n" "%8.4f\n"
        "\n"
        "[%.1f-%.1f]",
        d.cmd.thrust,
        d.cmd.torque.x(), d.cmd.torque.y(), d.cmd.torque.z(),
        d.motors[0], d.motors[1], d.motors[2], d.motors[3],
        motor_min < 999 ? motor_min : 0.0f, 
        motor_max > -999 ? motor_max : 0.0f);
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMRIGHT, vp, br_title, br_val, &con_);
}

void Visualizer::drawSimplePlot(int x, int y, int w, int h,
                                const std::deque<float>& data,
                                float min_val, float max_val,
                                const char* label) {
    // Simplified plot: just display range info in text
    (void)x; (void)y; (void)w; (void)h; (void)data; 
    (void)min_val; (void)max_val; (void)label; (void)&con_;
}

void Visualizer::render(const SimDisplay& d) {
    glfwPollEvents();

    int W, H;
    glfwGetFramebufferSize(window_, &W, &H);
    mjrRect vp{0, 0, W, H};

    mjv_updateScene(env_.model(), env_.data(), &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
    mjr_render(vp, &scn_, &con_);

    // Update history
    history_.push(d);
    
    // ── state overlays ────────────────────────────────────────────────────────
    renderTextOverlays(d, W, H);

    glfwSwapBuffers(window_);
}

// --- static GLFW callbacks ---

void Visualizer::keyCb(GLFWwindow* w, int key, int, int action, int) {
    if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
        glfwSetWindowShouldClose(w, GLFW_TRUE);
}

void Visualizer::scrollCb(GLFWwindow*, double, double dy) {
    if (instance_)
        instance_->cam_.distance = std::max(0.5, instance_->cam_.distance - dy * 0.3);
}

void Visualizer::mouseBtnCb(GLFWwindow*, int button, int action, int) {
    if (!instance_) return;
    instance_->btn_left_  = (glfwGetMouseButton(instance_->window_, GLFW_MOUSE_BUTTON_LEFT)  == GLFW_PRESS);
    instance_->btn_right_ = (glfwGetMouseButton(instance_->window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
}

void Visualizer::cursorCb(GLFWwindow*, double xpos, double ypos) {
    if (!instance_) return;
    double dx = xpos - instance_->last_x_;
    double dy = ypos - instance_->last_y_;
    instance_->last_x_ = xpos;
    instance_->last_y_ = ypos;

    if (instance_->btn_right_) {
        instance_->cam_.azimuth   += dx * 0.5;
        instance_->cam_.elevation -= dy * 0.5;
    }
    if (instance_->btn_left_) {
        double s = 0.003 * instance_->cam_.distance;
        instance_->cam_.lookat[0] -= dx * s;
        instance_->cam_.lookat[1] += dy * s;
    }
}

} // namespace sim
