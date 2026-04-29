#include "drone/trajectory.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

static void test_hover_trajectory() {
    drone::Vec3 pos(0, 0, 1);
    auto traj = drone::Trajectory::hover(pos, 10.0);
    assert((traj.getPosition(0.0)  - pos).norm() < 1e-9);
    assert((traj.getPosition(5.0)  - pos).norm() < 1e-9);
    assert((traj.getPosition(10.0) - pos).norm() < 1e-9);
    assert( traj.getVelocity(5.0).norm() < 1e-9);
    assert( traj.isComplete(10.1));
    assert(!traj.isComplete(9.9));
    std::cout << "[pass] hover trajectory\n";
}

static void test_linear_interpolation() {
    std::vector<drone::Waypoint> wps = {
        {{0, 0, 0}, {1, 0, 0}, 0.0},
        {{2, 0, 0}, {1, 0, 0}, 2.0}
    };
    drone::Trajectory traj(wps);
    drone::Vec3 mid = traj.getPosition(1.0);
    assert(std::abs(mid.x() - 1.0) < 1e-9 && "midpoint should be at x=1");
    std::cout << "[pass] linear interpolation midpoint = (" << mid.transpose() << ")\n";
}

static void test_circle_trajectory() {
    drone::Vec3 center(0, 0, 0);
    auto traj = drone::Trajectory::circle(center, 1.0, 0.0, 4.0, 4.0);
    // After one full period the position should return to start.
    drone::Vec3 p0  = traj.getPosition(0.0);
    drone::Vec3 pT  = traj.getPosition(4.0);
    assert((p0 - pT).norm() < 0.01 && "circle should close after one period");
    // Radius should be ~1 m throughout.
    for (int i = 0; i < 10; ++i) {
        drone::Vec3 p = traj.getPosition(4.0 * i / 10);
        double r = std::sqrt(p.x()*p.x() + p.y()*p.y());
        assert(std::abs(r - 1.0) < 0.01 && "circle radius should be 1 m");
    }
    std::cout << "[pass] circle trajectory radius = 1 m, closes after one period\n";
}

static void test_duration() {
    auto traj = drone::Trajectory::hover({0,0,1}, 15.0);
    assert(std::abs(traj.duration() - 15.0) < 1e-9);
    std::cout << "[pass] trajectory duration = " << traj.duration() << " s\n";
}

int main() {
    test_hover_trajectory();
    test_linear_interpolation();
    test_circle_trajectory();
    test_duration();
    std::cout << "All trajectory tests passed.\n";
    return 0;
}
