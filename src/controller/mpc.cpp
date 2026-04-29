#include "drone/controller/controller_base.hpp"

// MPC controller — not yet implemented.
// Placeholder satisfies the build; replace with a full MPC formulation later.

namespace drone
{
    class MpcController : public ControllerBase
    {
    public:
        explicit MpcController(double mass = 1.325, double gravity = 9.81)
            : mass_(mass), gravity_(gravity) {}

        ControlInput compute(const DroneState&, const Vec3&, const Vec3&) override
        {
            // Fallback: maintain hover thrust with zero torque.
            return {mass_ * gravity_, Vec3::Zero()};
        }

        void reset() override {}

    private:
        double mass_;
        double gravity_;
    };
} // namespace drone