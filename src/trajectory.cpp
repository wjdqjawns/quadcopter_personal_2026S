#include "drone/trajectory.hpp"
#include <cmath>
#include <stdexcept>

namespace drone
{
    Trajectory::Trajectory(std::vector<Waypoint> waypoints) : wps_(std::move(waypoints))
    {
        if (wps_.size() < 2)
            throw std::invalid_argument("Trajectory needs at least 2 waypoints");
    }

    double Trajectory::duration() const
    {
        return wps_.back().arrival_time;
    }

    bool Trajectory::isComplete(double t) const
    { 
        return t >= duration();
    }

    std::pair<int,double> Trajectory::locate(double t) const
    {
        t = std::clamp(t, 0.0, duration());
        for (int i = 0; i + 1 < static_cast<int>(wps_.size()); ++i)
        {
            double t0 = wps_[i].arrival_time;
            double t1 = wps_[i+1].arrival_time;
            
            if (t <= t1)
            {
                double alpha = (t1 > t0) ? (t - t0) / (t1 - t0) : 1.0;
                return {i, alpha};
            }
        }
        return {static_cast<int>(wps_.size()) - 2, 1.0};
    }

    Vec3 Trajectory::getPosition(double t) const
    {
        auto [i, a] = locate(t);
        return (1.0 - a) * wps_[i].pos + a * wps_[i+1].pos;
    }

    Vec3 Trajectory::getVelocity(double t) const
    {
        auto [i, a] = locate(t);
        return (1.0 - a) * wps_[i].vel + a * wps_[i+1].vel;
    }

    // --- factory methods ---

    Trajectory Trajectory::hover(const Vec3& pos, double dur)
    {
        return Trajectory({{pos, Vec3::Zero(), 0.0}, {pos, Vec3::Zero(), dur}});
    }

    Trajectory Trajectory::circle(const Vec3& center, double radius, double height, double period, double dur)
    {
        const int N = 48;
        std::vector<Waypoint> wps;
        wps.reserve(N + 1);
        const double omega = 2.0 * M_PI / period;
        
        for (int i = 0; i <= N; ++i)
        {
            double t   = dur * i / N;
            double phi = omega * t;
            Vec3 pos(center.x() + radius * std::cos(phi),
                    center.y() + radius * std::sin(phi),
                    center.z() + height);
            Vec3 vel(-radius * omega * std::sin(phi),
                    radius * omega * std::cos(phi),
                    0.0);
            wps.push_back({pos, vel, t});
        }

        return Trajectory(std::move(wps));
    }
} // namespace drone
