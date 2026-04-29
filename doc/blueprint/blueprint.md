# Drone Simulation — System Blueprint

## 1. Overview

This project is a **physics-accurate quadrotor simulation** built on MuJoCo 3.x.
It models a Skydio X2-geometry drone and supports two interchangeable flight
controllers (PID and MPC), state estimation (EKF + Complementary Filter),
IMU noise injection, and live 3-D visualisation.

---

## 2. High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                          main.cpp  (simulation loop @ 100 Hz)       │
│                                                                     │
│   ┌─────────┐    raw IMU    ┌──────────────┐   estimated state      │
│   │MuJoCo   │──────────────▶│ SensorBridge │──────────────────┐     │
│   │  Env    │               └──────────────┘                  │     │
│   │(physics)│                                                 ▼     │
│   │         │◀─ motor cmds ─┐  ┌─────────────────────────────────┐  │
│   └──────┬──┘               │  │  State Estimator Pipeline       │  │
│          │ ground truth     │  │  CF (attitude) → EKF (pos/vel)  │  │
│          │                  │  └────────────────┬────────────────┘  │
│          ▼                  │                   │ DroneState        │
│   ┌──────────────┐  motors  │  ┌────────────────▼────────────────┐  │
│   │  Visualizer  │          └──│     Controller (PID or MPC)     │  │
│   │  (GLFW/OGL)  │             │  pos/vel → thrust + torques     │  │
│   └──────────────┘             └─────────────────────────────────┘  │
│                                         ▲                           │
│                               ┌─────────┴──────────┐                │
│                               │    Trajectory      │                │
│                               │ (waypoint splines) │                │
│                               └────────────────────┘                │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 3. Module Breakdown

### 3.1 Physics Engine — `sim/mujoco_env.*`

| Responsibility | Detail |
|---|---|
| Load model XML | `asset/skydio_x2/scene.xml` |
| Step physics | `mj_step()` at dt = 0.01 s |
| Expose actuators | `setActuators(Vec4 motor_forces)` → clamp to [0, 13] N |
| Expose sensors | `getSensorGyro()`, `getSensorAccel()`, `getSensorQuat()` |
| Expose ground truth | `getPosition()`, `getVelocity()`, `getOrientation()`, `getAngularVelocity()` |

### 3.2 Sensor Bridge — `sim/sensor_bridge.*`

Thin adapter between `MujocoEnv` raw outputs and domain types.

- `getGroundTruth()` → `DroneState` (position, velocity, quaternion, angular velocity)
- `getRawIMU(dt)` → `IMUData` (specific force + angular velocity from MuJoCo sensors)

### 3.3 IMU Noise Model — `src/imu/`, `include/drone/imu.hpp`

Applies configurable white noise + constant bias to raw IMU data before feeding estimators.

| Parameter | Default | Unit |
|---|---|---|
| `accel_noise` | 0.003 | m/s²/√Hz |
| `gyro_noise`  | 0.0002 | rad/s/√Hz |
| `accel_bias`  | 0.01 | m/s² |
| `gyro_bias`   | 0.001 | rad/s |

### 3.4 State Estimators — `src/estimator/`, `include/drone/estimator/`

Two estimators run in series each step:

**Complementary Filter (`cf.*`)**
- Fuses gyro integration with accelerometer-derived roll/pitch.
- Alpha (gyro weight) = 0.98 by default.
- Output: orientation quaternion + angular velocity.

**Extended Kalman Filter (`ekf.*`)**
- 6-state linear Kalman filter: `[pos(3), vel(3)]`.
- Prediction: IMU specific force rotated to world frame → double integrator.
- Orientation injected from CF (`setOrientation()`).
- Optional measurement update: `fusePosition()` for GPS/mocap.

### 3.5 Controllers — `src/controller/`, `include/drone/controller/`

Both controllers implement `ControllerBase`:

```cpp
ControlInput compute(const DroneState& state,
                     const Vec3& pos_des, const Vec3& vel_des);
```

**PID Controller (`pid.*`)**

Cascade structure:
1. Position PD → desired world-frame acceleration `a_des`
2. `a_des` → desired roll/pitch attitude
3. Attitude PD → `torque`
4. `motorMix()` → per-motor forces

**MPC Controller (`mpc.*`)**

Outer loop uses unconstrained finite-horizon linear MPC on a double-integrator position model:

| Symbol | Meaning |
|---|---|
| `x = [pos, vel]` | 6-D state |
| `u = a_des` | 3-D input (world-frame acceleration) |
| `N` | Prediction horizon (default 10 steps = 0.1 s) |
| `Q` | Diagonal state cost `[10,10,20,4,4,6]` |
| `R` | Diagonal input cost `[0.1,0.1,0.1]` |

Gain matrix `K_opt` is precomputed once in the constructor (closed-form batch LQR).
The inner attitude loop is identical to PID.

### 3.6 Trajectory — `src/trajectory.cpp`, `include/drone/trajectory.hpp`

Piecewise-linear interpolation over a list of `Waypoint{pos, vel, arrival_time}`.

Factory methods:
- `Trajectory::hover(pos, duration)` — stationary hover
- `Trajectory::circle(center, radius, height, period, duration)` — horizontal circle

### 3.7 Visualiser — `sim/visualizer.*`

MuJoCo passive viewer with GLFW + OpenGL overlays:
- 3-D rendered drone scene with camera tracking.
- Text overlays: ground truth, EKF estimate, target, control outputs.
- `PlotHistory` deque buffer (1 000 frames ≈ 10 s).

---

## 4. Data Flow Per Step

```
MuJoCo sensors
     │
     ├─ raw accelerometer + gyro
     │        │
     │    ImuNoise::apply()
     │        │
     │   CF::update()     → attitude (q, ω)
     │        │
     │   EKF::setOrientation()
     │   EKF::update()    → position, velocity
     │        │
     │   Controller::compute(est, pos_ref, vel_ref)
     │        │
     │   motorMix()       → [m1, m2, m3, m4]
     │        │
     └─ MujocoEnv::setActuators() → mj_step()
```

---

## 5. Motor Allocation (X-configuration)

Site positions (body frame, x = forward, y = left):

| Motor | x (m) | y (m) | yaw sign |
|---|---|---|---|
| m1 | −0.14 | −0.18 | − |
| m2 | −0.14 | +0.18 | + |
| m3 | +0.14 | +0.18 | − |
| m4 | +0.14 | −0.18 | + |

Allocation inverse (equal arm symmetry):

```
m_i = F/4  ±  τx/(4·arm_y)  ±  τy/(4·arm_x)  ±  τz/(4·kq)
```

---

## 6. Configuration Files

| File | Controls |
|---|---|
| `config/sim/skydio_x2.yaml` | model path, duration, visualize flag, **controller selection** |
| `config/controller_params.yaml` | PID gains, MPC weights / horizon |
| `config/estimator_params.yaml` | EKF noise, CF alpha |
| `config/imu_params.yaml` | IMU noise / bias |

---

## 7. Directory Structure

```
prj_drone/
├── asset/skydio_x2/        Skydio X2 MuJoCo model (XML + OBJ)
├── config/                 YAML parameter files
│   ├── sim/
│   └── *.yaml
├── doc/                    This documentation
├── include/drone/          Public C++ headers
│   ├── controller/         controller_base, pid, mpc
│   ├── estimator/          estimator_base, ekf, cf
│   ├── imu.hpp, trajectory.hpp, utils.hpp
├── scripts/                Python post-processing & plotting
├── sim/                    MuJoCo adapter layer (mujoco_env, sensor_bridge, visualizer)
├── src/                    Implementation files
│   ├── controller/         pid.cpp, mpc.cpp
│   ├── estimator/          ekf.cpp, cf.cpp
│   ├── imu/                imu.cpp, imu_noise.cpp
│   ├── main.cpp
│   └── trajectory.cpp
└── test/                   Unit tests per module
```
