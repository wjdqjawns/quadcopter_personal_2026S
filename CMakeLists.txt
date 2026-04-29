cmake_minimum_required(VERSION 3.18)
project(prj_drone)

find_package(mujoco REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

file(GLOB CONTROLLER_SRCS src/controller/*.cpp)
file(GLOB ESTIMATOR_SRCS src/estimator/*.cpp)
file(GLOB IMU_SRCS       src/imu/*.cpp)

add_executable(drone_sim
  src/main.cpp
  src/trajectory.cpp
  ${CONTROLLER_SRCS}
  ${ESTIMATOR_SRCS}
  ${IMU_SRCS}
  sim/mujoco_env.cpp
  sim/sensor_bridge.cpp
)
target_link_libraries(drone_sim mujoco::mujoco Eigen3::Eigen yaml-cpp)
target_include_directories(drone_sim PRIVATE include)