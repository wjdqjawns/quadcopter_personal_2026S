#pragma once

#include "drone/utils.hpp"
#include <vector>

namespace drone
{
    struct Waypoint
    {
        Vec3   pos;
        Vec3   vel;          // feedforward velocity at this waypoint
        double arrival_time; // seconds from trajectory start
    };

    class Trajectory
    {
    public:
        explicit Trajectory(std::vector<Waypoint> waypoints);

        Vec3   getPosition(double t) const;
        Vec3   getVelocity(double t) const;
        bool   isComplete(double t)  const;
        double duration()            const;

        static Trajectory hover(const Vec3& pos, double duration);
        static Trajectory circle(const Vec3& center, double radius,
                                double height, double period, double duration);

    private:
        std::vector<Waypoint> wps_;
        std::pair<int,double> locate(double t) const; // {segment_idx, alpha∈[0,1]}
    };
} // namespace drone
