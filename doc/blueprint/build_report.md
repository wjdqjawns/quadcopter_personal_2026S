# Technical Build Report

## 1. Project Identification

| Field | Value |
|---|---|
| Project name | `prj_drone` |
| Language | C++17 |
| Build system | CMake 3.18+ |
| Physics engine | MuJoCo 3.x |
| Primary target | `drone_sim` (simulation binary) |
| Test targets | `test_controller`, `test_estimator`, `test_imu`, `test_trajectory` |

---

## 2. Dependency Graph

```
drone_sim
├── mujoco::mujoco       physics simulation
├── Eigen3::Eigen        linear algebra (vectors, matrices, quaternions)
├── yaml-cpp             config file parsing
├── glfw                 window / input / OpenGL context
└── OpenGL::GL           3-D rendering

test_controller          Eigen3 only
test_estimator           Eigen3 only
test_imu                 Eigen3 only
test_trajectory          Eigen3 only
```

---

## 3. Source File Map

### 3.1 Headers (`include/drone/`)

| File | Declares |
|---|---|
| `utils.hpp` | `Vec3`, `Vec4`, `Mat3`, `Quat`, `IMUData`, `DroneState`, `ControlInput`, `quatToEuler`, `eulerToQuat` |
| `imu.hpp` | `ImuNoiseParams`, `ImuNoise`, `Imu` |
| `trajectory.hpp` | `Waypoint`, `Trajectory` |
| `controller/controller_base.hpp` | `ControllerBase` (pure virtual) |
| `controller/pid.hpp` | `PidParams`, `PidController`, `motorMix()` |
| `controller/mpc.hpp` | `MpcParams`, `MpcController`, `mpcMotorMix()` |
| `estimator/estimator_base.hpp` | `EstimatorBase` (pure virtual) |
| `estimator/ekf.hpp` | `EkfParams`, `EkfEstimator` |
| `estimator/cf.hpp` | `CfParams`, `CfEstimator` |

### 3.2 Implementation (`src/`)

| File | Implements |
|---|---|
| `main.cpp` | Simulation loop, config loading, controller selection |
| `trajectory.cpp` | Waypoint interpolation, circle factory |
| `controller/pid.cpp` | PID cascade control + motor allocation |
| `controller/mpc.cpp` | Batch LQR MPC — matrix precomputation + per-step solve |
| `estimator/ekf.cpp` | 6-state Kalman filter (predict + position fuse) |
| `estimator/cf.cpp` | Complementary filter for attitude |
| `imu/imu.cpp` | `Imu::process()` wrapper |
| `imu/imu_noise.cpp` | White noise + bias injection |

### 3.3 Simulation Layer (`sim/`)

| File | Purpose |
|---|---|
| `mujoco_env.cpp/.hpp` | MuJoCo model load, step, sensor read, actuator write |
| `sensor_bridge.cpp/.hpp` | Adapter: MuJoCo data → `DroneState` / `IMUData` |
| `visualizer.cpp/.hpp` | GLFW window, MuJoCo passive viewer, text overlays |

---

## 4. CMake Configuration

### 4.1 Key Variables

| Variable | Default | Override |
|---|---|---|
| `MUJOCO_ROOT` | `$HOME/mujoco/install` | `-DMUJOCO_ROOT=<path>` |
| `CMAKE_BUILD_TYPE` | (none) | `-DCMAKE_BUILD_TYPE=Release` |
| `CMAKE_CXX_STANDARD` | 17 | fixed in CMakeLists.txt |

### 4.2 Important Flags

- `MUJOCO_PLUGIN_DIR` compile definition: baked into the binary so that
  MuJoCo can locate OBJ/STL decoder plugins at runtime without
  `LD_LIBRARY_PATH`.
- `CMAKE_INSTALL_RPATH` and `CMAKE_BUILD_RPATH`: set to `${MUJOCO_ROOT}/lib`
  so the binary finds `libmujoco.so` without manual env-var setup.

### 4.3 Source Globs

```cmake
file(GLOB CONTROLLER_SRCS src/controller/*.cpp)
file(GLOB ESTIMATOR_SRCS  src/estimator/*.cpp)
file(GLOB IMU_SRCS        src/imu/*.cpp)
```

New `.cpp` files placed in these directories are automatically compiled.

---

## 5. Build Steps (Detailed)

### 5.1 First-time Setup

```bash
# 1. Install system dependencies (Ubuntu 22.04)
sudo apt update
sudo apt install -y build-essential cmake libeigen3-dev \
                    libyaml-cpp-dev libglfw3-dev libgl1-mesa-dev

# 2. Install MuJoCo (if not already present)
#    Download the Linux tar from https://github.com/google-deepmind/mujoco/releases
mkdir -p ~/mujoco/install
tar -xzf mujoco-3.x.x-linux-x86_64.tar.gz -C ~/mujoco/install --strip-components=1

# 3. Clone / enter the repo
cd /path/to/prj_drone

# 4. Configure CMake (Release build)
cmake -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DMUJOCO_ROOT=$HOME/mujoco/install

# 5. Compile (use all available cores)
cmake --build build -j$(nproc)
```

### 5.2 Incremental Rebuild

```bash
cmake --build build -j$(nproc)
```

CMake tracks file dependencies; only changed translation units are recompiled.

### 5.3 Debug Build

```bash
cmake -B build_debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build_debug -j$(nproc)
# Run with GDB:
gdb --args ./build_debug/drone_sim
```

### 5.4 Clean Build

```bash
rm -rf build
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

---

## 6. Runtime Requirements

| Requirement | For |
|---|---|
| `libmujoco.so.3.x` on RPATH | All targets (baked in at build time) |
| X11 / Wayland display | `drone_sim` with `visualize: true` |
| `asset/skydio_x2/scene.xml` | Must be present relative to working dir |
| `config/` YAML files | Loaded at runtime; safe to modify without rebuilding |
| `data/logs/` directory | Created automatically if absent? No — **create it manually** |

```bash
mkdir -p data/logs
```

---

## 7. Expected Build Output

```
-- The CXX compiler identification is GNU 11.x
-- Detecting CXX compiler ABI info...
-- Found mujoco: ...
-- Found Eigen3: ...
-- Found yaml-cpp: ...
-- Found glfw3: ...
-- Found OpenGL: ...
-- Configuring done
-- Build files have been written to: .../prj_drone/build

[  8%] Building CXX object CMakeFiles/drone_sim.dir/src/main.cpp.o
[  16%] Building CXX object CMakeFiles/drone_sim.dir/src/trajectory.cpp.o
[  24%] Building CXX object CMakeFiles/drone_sim.dir/src/controller/pid.cpp.o
[  32%] Building CXX object CMakeFiles/drone_sim.dir/src/controller/mpc.cpp.o
[  40%] Building CXX object CMakeFiles/drone_sim.dir/src/estimator/ekf.cpp.o
[  48%] Building CXX object CMakeFiles/drone_sim.dir/src/estimator/cf.cpp.o
[  56%] Building CXX object CMakeFiles/drone_sim.dir/src/imu/imu.cpp.o
[  64%] Building CXX object CMakeFiles/drone_sim.dir/src/imu/imu_noise.cpp.o
[  72%] Building CXX object CMakeFiles/drone_sim.dir/sim/mujoco_env.cpp.o
[  80%] Building CXX object CMakeFiles/drone_sim.dir/sim/sensor_bridge.cpp.o
[  88%] Building CXX object CMakeFiles/drone_sim.dir/sim/visualizer.cpp.o
[  96%] Linking CXX executable drone_sim
[100%] Built target drone_sim
```

---

## 8. Compile-time Checks

After building, verify the RPATH and plugin path are correct:

```bash
# Check RPATH embeds MuJoCo lib path
readelf -d build/drone_sim | grep RPATH

# Confirm plugin path is baked in
strings build/drone_sim | grep mujoco_plugin
```

---

## 9. Running Tests

```bash
./build/test_controller   # should print PASS for all cases
./build/test_estimator
./build/test_imu
./build/test_trajectory
```

---

## 10. Troubleshooting

### `libmujoco.so.3: cannot open shared object file`
MuJoCo was installed to a non-standard location.  
Set `MUJOCO_ROOT` during CMake configuration to match the actual install path.

### `error: /usr/bin/ld: cannot find -lmujoco`
The `mujoco` CMake package is not found.  
Verify `$MUJOCO_ROOT/lib/cmake/mujoco/mujocoConfig.cmake` exists.

### `yaml-cpp: target not found`
Install `libyaml-cpp-dev` or provide the install prefix:
```bash
cmake -B build -Dyaml-cpp_DIR=/path/to/yaml-cpp/lib/cmake/yaml-cpp
```

### Visualizer window does not open
Set `DISPLAY` or disable visualization:
```yaml
simulation:
  visualize: false
```

### MPC matrices are large / slow to build
Increase N with caution. At N=10 (default), `K_opt` is 30×60 — trivial to
compute once. At N=50 it becomes 150×300 — still acceptable at startup.
Reduce N if startup latency is a concern.
