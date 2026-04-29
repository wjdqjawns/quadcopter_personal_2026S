# QUADCOPTER — Quadrotor Flight Simulation

A physics-accurate quadrotor simulation built on **MuJoCo 3.x** with a
Skydio X2 drone model.
Supports two interchangeable flight controllers (**PID** and **MPC**),
sensor-realistic state estimation (EKF + Complementary Filter),
IMU noise injection, live 3-D visualisation, and CSV trajectory logging.

---

## Features

- **MuJoCo physics** — full rigid-body dynamics, aerodynamic drag, motor inertia
- **PID controller** — cascade position-PD → attitude-PD, proven baseline
- **MPC controller** — finite-horizon linear MPC (unconstrained batch LQR)
  with precomputed gain matrix; inner attitude loop shared with PID
- **State estimation** — Complementary Filter (attitude) + EKF (position/velocity)
- **IMU noise model** — configurable white noise and constant bias
- **Trajectory planner** — piecewise-linear waypoints; circle and hover factories
- **Live visualiser** — MuJoCo 3-D view with GLFW overlays (ground truth, EKF, control outputs)
- **CSV logging** — full state, estimation, and control data at 100 Hz
- **Python scripts** — trajectory plot, Kalman plot, log statistics

---

## Prerequisites

| Package | Ubuntu package | Min version |
|---|---|---|
| CMake | `cmake` | 3.18 |
| Eigen3 | `libeigen3-dev` | 3.3 |
| yaml-cpp | `libyaml-cpp-dev` | 0.6 |
| GLFW3 | `libglfw3-dev` | 3.3 |
| OpenGL | `libgl1-mesa-dev` | any |
| MuJoCo | (see below) | 3.x |

```bash
sudo apt install build-essential cmake libeigen3-dev \
                 libyaml-cpp-dev libglfw3-dev libgl1-mesa-dev
```

**MuJoCo 3.x** — download the Linux release and extract to `~/mujoco/install`:

```bash
mkdir -p ~/mujoco/install
tar -xzf mujoco-3.x.x-linux-x86_64.tar.gz -C ~/mujoco/install --strip-components=1
```

If you install to a different path, pass `-DMUJOCO_ROOT=<path>` to CMake.

---

## Build

```bash
# Configure
cmake -B build -DCMAKE_BUILD_TYPE=Release

# Compile all targets
cmake --build build -j$(nproc)

# Create required output directory
mkdir -p data/logs
```

---

## Quick Start

```bash
# Run with default config (PID controller, visualisation enabled)
./build/drone_sim

# Run with MPC controller (edit config first)
# In config/sim/skydio_x2.yaml, set: controller: mpc
./build/drone_sim
```

The simulation runs a takeoff → 1 m hover → 1 m radius circle → land mission
for 60 seconds and saves a CSV log to `data/logs/sim_log.csv`.

---

## Switching Between PID and MPC

Edit `config/sim/skydio_x2.yaml`:

```yaml
simulation:
  controller: mpc   # options: pid | mpc
```

Controller parameters are in `config/controller_params.yaml`:

```yaml
pid:
  kp_pos: [6, 6, 8]
  kd_pos: [4, 4, 5]
  # ...

mpc:
  horizon: 10          # prediction steps (0.1 s lookahead at 100 Hz)
  Q: [10,10,20,4,4,6]  # state cost [px,py,pz,vx,vy,vz]
  R: [0.1,0.1,0.1]     # input cost [ax,ay,az]
  # ...
```

---

## Headless Mode (No Display)

```yaml
# config/sim/skydio_x2.yaml
simulation:
  visualize: false
```

---

## Analysing Results

```bash
# 3-D trajectory plot
python3 scripts/plot_trajectory.py

# EKF estimate vs ground truth
python3 scripts/plot_kalman.py

# Summary statistics
python3 scripts/analyze_log.py
```

---

## Running Unit Tests

```bash
./build/test_controller   # PID gain response
./build/test_estimator    # EKF convergence
./build/test_imu          # noise/bias injection
./build/test_trajectory   # interpolation correctness
```

---

## Project Structure

```
prj_drone/
├── asset/skydio_x2/        MuJoCo drone model (XML + OBJ + textures)
├── config/
│   ├── sim/skydio_x2.yaml  simulation settings & controller selection
│   ├── controller_params.yaml  PID gains, MPC weights
│   ├── estimator_params.yaml
│   └── imu_params.yaml
├── doc/
│   ├── blueprint.md        system architecture
│   ├── dev_guide.md        developer reference
│   └── build_report.md     build instructions & dependency details
├── include/drone/          public C++ headers
├── scripts/                Python post-processing
├── sim/                    MuJoCo adapter (env, sensor bridge, visualizer)
├── src/                    implementation
│   ├── controller/         pid.cpp, mpc.cpp
│   ├── estimator/          ekf.cpp, cf.cpp
│   ├── imu/
│   ├── main.cpp
│   └── trajectory.cpp
└── test/                   unit tests
```

---

## MPC Architecture Summary

The MPC outer loop solves an unconstrained finite-horizon LQR problem:

```
State:   x = [pos, vel]  (6-D)
Input:   u = a_des       (3-D world-frame acceleration)
Model:   x_{k+1} = A x_k + B u_k    (discrete double integrator, dt = 0.01 s)

Cost:    J = Σ (x_k - x_ref)^T Q (x_k - x_ref) + u_k^T R u_k

Optimal: U* = K_opt (X_ref - Phi x_0)     [closed form, computed once]
```

The first element of `U*` (desired acceleration `a_des`) is passed to the
same attitude inner loop used by PID (roll/pitch setpoint → PD torque).

---

## Documentation

Full documentation is in the `doc/` folder:

- [`doc/blueprint/blueprint.md`](doc/blueprint.md) — system architecture and data-flow diagrams
- [`doc/blueprint/dev_guide.md`](doc/dev_guide.md) — how to extend controllers, estimators, trajectories
- [`doc/blueprint/build_report.md`](doc/build_report.md) — detailed build steps and troubleshooting