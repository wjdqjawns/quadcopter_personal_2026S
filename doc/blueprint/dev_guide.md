# Developer Guide

## Prerequisites

| Tool | Minimum version | Notes |
|---|---|---|
| GCC / Clang | C++17 support | GCC 9+, Clang 10+ |
| CMake | 3.18 | `cmake --version` |
| MuJoCo | 3.x | Install to `~/mujoco/install` or set `MUJOCO_ROOT` |
| Eigen3 | 3.3 | `libeigen3-dev` on Ubuntu |
| yaml-cpp | 0.6 | `libyaml-cpp-dev` |
| GLFW3 | 3.3 | `libglfw3-dev` |
| OpenGL | any | `libgl1-mesa-dev` |
| Python 3 (optional) | 3.8+ | for analysis scripts |

Install on Ubuntu 22.04:
```bash
sudo apt install build-essential cmake libeigen3-dev libyaml-cpp-dev \
                 libglfw3-dev libgl1-mesa-dev python3-pip
pip3 install numpy matplotlib pandas
```

---

## Build

```bash
cd /path/to/prj_drone

# Configure (first time only)
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Override MuJoCo location if not in ~/mujoco/install
cmake -B build -DMUJOCO_ROOT=/opt/mujoco

# Build all targets
cmake --build build -j$(nproc)
```

Binaries produced:

| Binary | Purpose |
|---|---|
| `build/drone_sim` | Main simulation |
| `build/test_controller` | Controller unit tests |
| `build/test_estimator` | Estimator unit tests |
| `build/test_imu` | IMU noise unit tests |
| `build/test_trajectory` | Trajectory unit tests |

---

## Running the Simulation

```bash
# From repo root (so relative config paths resolve correctly)
./build/drone_sim                          # default config
./build/drone_sim config/sim/skydio_x2.yaml  # explicit config
```

### Switch Between PID and MPC

Edit `config/sim/skydio_x2.yaml`:

```yaml
simulation:
  controller: mpc   # or pid
```

Or pass a modified config file:
```bash
./build/drone_sim config/sim/skydio_x2_mpc.yaml
```

### Headless (no display)

```yaml
# config/sim/skydio_x2.yaml
simulation:
  visualize: false
```

---

## Adding a New Controller

1. Create `include/drone/controller/my_ctrl.hpp` — extend `ControllerBase`:

```cpp
#pragma once
#include "drone/controller/controller_base.hpp"

namespace drone {

struct MyCtrlParams { /* ... */ };

class MyController : public ControllerBase {
public:
    explicit MyController(const MyCtrlParams& p = {});
    ControlInput compute(const DroneState& state,
                         const Vec3& pos_des,
                         const Vec3& vel_des) override;
    void reset() override;
private:
    MyCtrlParams p_;
};

} // namespace drone
```

2. Implement in `src/controller/my_ctrl.cpp`.  
   The CMake glob `file(GLOB CONTROLLER_SRCS src/controller/*.cpp)` picks it up
   automatically — no `CMakeLists.txt` change needed.

3. Add a `my_ctrl` branch in `src/main.cpp` (copy the `pid` branch pattern):

```cpp
} else if (ctrl_type == "my_ctrl") {
    ctrl      = std::make_unique<drone::MyController>(my_params);
    motor_mix = [my_params](const drone::ControlInput& cmd) { ... };
}
```

4. Add `controller: my_ctrl` to the YAML sim config and run.

---

## Adding a New Estimator

1. Extend `EstimatorBase` (`include/drone/estimator/estimator_base.hpp`):
   - `update(const IMUData&)`
   - `getState() const → DroneState`
   - `reset(const DroneState&)`

2. Wire it into the estimator pipeline in `src/main.cpp` (currently CF → EKF).

---

## Modifying the Trajectory

`drone::Trajectory` accepts any `std::vector<Waypoint>`. Each waypoint has:

```cpp
struct Waypoint {
    Vec3   pos;           // world-frame position (m)
    Vec3   vel;           // feedforward velocity (m/s)
    double arrival_time;  // seconds from trajectory start
};
```

Between waypoints, position and velocity are linearly interpolated.
Use `Trajectory::circle(...)` or `Trajectory::hover(...)` as factory helpers.

---

## Configuration System

All parameters are loaded at startup from YAML files in `config/`.
None are hard-coded in the binaries.

| Parameter group | File | Key prefix |
|---|---|---|
| Simulation | `config/sim/skydio_x2.yaml` | `simulation.` |
| PID gains | `config/controller_params.yaml` | `pid.` |
| MPC weights | `config/controller_params.yaml` | `mpc.` |
| EKF noise | `config/estimator_params.yaml` | `ekf.` |
| CF alpha | `config/estimator_params.yaml` | `complementary_filter.` |
| IMU noise | `config/imu_params.yaml` | `imu.` |

If a config file is missing or a key is absent, defaults from the corresponding
`*Params` struct are used and a `[warn]` message is printed to stderr.

---

## Log File Format

`data/logs/sim_log.csv` — one row per simulation step (100 Hz):

| Column | Unit | Source |
|---|---|---|
| `t` | s | simulation time |
| `px,py,pz` | m | ground truth position |
| `vx,vy,vz` | m/s | ground truth velocity |
| `roll,pitch,yaw` | rad | ground truth Euler angles (ZYX) |
| `thrust` | N | controller output |
| `tau_x,tau_y,tau_z` | N·m | controller torques |
| `m1..m4` | N | motor force commands |
| `est_px..est_vz` | m, m/s | EKF estimated position + velocity |

---

## Analysis Scripts

```bash
# 3-D trajectory plot
python3 scripts/plot_trajectory.py

# EKF estimate vs ground truth
python3 scripts/plot_kalman.py

# Print summary statistics
python3 scripts/analyze_log.py
```

All scripts read `data/logs/sim_log.csv` by default.

---

## Unit Tests

```bash
./build/test_controller   # PID gain response check
./build/test_estimator    # EKF convergence check
./build/test_imu          # noise / bias injection check
./build/test_trajectory   # interpolation check
```

---

## MPC Tuning Guide

The MPC controller (outer loop) is governed by five knobs:

| Parameter | YAML key | Effect |
|---|---|---|
| Horizon `N` | `mpc.horizon` | Longer = smoother but slower to react; default 10 |
| Position weights `Q[0:3]` | `mpc.Q[0..2]` | Higher = tighter position tracking |
| Velocity weights `Q[3:6]` | `mpc.Q[3..5]` | Higher = damps oscillation |
| Input cost `R` | `mpc.R` | Higher = softer acceleration commands |
| Attitude gains | `mpc.att.kp/kd` | Inner-loop bandwidth |

Typical tuning sequence:
1. Start with large `R` (0.5–1.0) for smooth commands, verify stability.
2. Reduce `R` until tracking error is acceptable.
3. Increase `Q[0:3]` if position steady-state error is too large.
4. Increase `Q[3:6]` if velocity oscillates.
5. Tune `mpc.att.kp/kd` if attitude response lags thrust commands.

---

## Common Issues

| Symptom | Likely cause | Fix |
|---|---|---|
| `Cannot load config/…` | Wrong working directory | Run from repo root |
| `glfwInit failed` | No display | Set `visualize: false` in YAML |
| Drone flips immediately | Wrong `mass` in params | Check `pid.mass` / `mpc.mass` matches `x2.xml` |
| MPC hover oscillation | R too small | Increase `mpc.R` values |
| High estimation drift | Bad CF alpha | Lower `alpha` for more accel correction |
