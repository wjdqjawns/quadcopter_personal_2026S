cmake_minimum_required(VERSION 3.18)
project(prj_drone CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MuJoCo is built from source at ~/mujoco/install.
# Override with -DMUJOCO_ROOT=<path> if installed elsewhere.
set(MUJOCO_ROOT "$ENV{HOME}/mujoco/install" CACHE PATH "MuJoCo install prefix")
list(APPEND CMAKE_PREFIX_PATH "${MUJOCO_ROOT}")

find_package(mujoco  REQUIRED)
find_package(Eigen3  REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(glfw3   REQUIRED)
find_package(OpenGL  REQUIRED)

# Keep the binary runnable without LD_LIBRARY_PATH.
set(CMAKE_INSTALL_RPATH "${MUJOCO_ROOT}/lib")
set(CMAKE_BUILD_RPATH   "${MUJOCO_ROOT}/lib")

# ── source globs ─────────────────────────────────────────────────────────────
file(GLOB CONTROLLER_SRCS src/controller/*.cpp)
file(GLOB ESTIMATOR_SRCS  src/estimator/*.cpp)
file(GLOB IMU_SRCS        src/imu/*.cpp)

# ── main simulation binary ───────────────────────────────────────────────────
add_executable(drone_sim
  src/main.cpp
  src/trajectory.cpp
  ${CONTROLLER_SRCS}
  ${ESTIMATOR_SRCS}
  ${IMU_SRCS}
  sim/mujoco_env.cpp
  sim/sensor_bridge.cpp
  sim/visualizer.cpp
)
target_link_libraries(drone_sim
  mujoco::mujoco Eigen3::Eigen yaml-cpp glfw OpenGL::GL)
target_include_directories(drone_sim PRIVATE include sim)
# Bake plugin directory path so mujoco_env.cpp can load the OBJ/STL decoders.
target_compile_definitions(drone_sim PRIVATE
  MUJOCO_PLUGIN_DIR="${MUJOCO_ROOT}/bin/mujoco_plugin"
)

# ── unit tests ───────────────────────────────────────────────────────────────
set(DRONE_CORE_SRCS
  src/trajectory.cpp
  ${CONTROLLER_SRCS}
  ${ESTIMATOR_SRCS}
  ${IMU_SRCS}
)

add_executable(test_controller
  test/controller/test_controller.cpp
  src/controller/pid.cpp
)
target_link_libraries(test_controller Eigen3::Eigen)
target_include_directories(test_controller PRIVATE include)

add_executable(test_estimator
  test/estimator/test_estimator.cpp
  src/estimator/ekf.cpp
  src/estimator/cf.cpp
)
target_link_libraries(test_estimator Eigen3::Eigen)
target_include_directories(test_estimator PRIVATE include)

add_executable(test_imu
  test/imu/test_imu.cpp
  src/imu/imu.cpp
  src/imu/imu_noise.cpp
)
target_link_libraries(test_imu Eigen3::Eigen)
target_include_directories(test_imu PRIVATE include)

add_executable(test_trajectory
  test/trajectory/test_trajectory.cpp
  src/trajectory.cpp
)
target_link_libraries(test_trajectory Eigen3::Eigen)
target_include_directories(test_trajectory PRIVATE include)