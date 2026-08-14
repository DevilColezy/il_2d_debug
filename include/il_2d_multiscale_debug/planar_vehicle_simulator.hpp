#pragma once
/// @file   planar_vehicle_simulator.hpp
/// @brief  Fixed-step (dt = 1/30 s) deterministic planar vehicle model.
///
/// Applies the 30 Hz planner's body-velocity + yaw-rate command under:
///   * max speed, max linear acceleration,
///   * max yaw rate, max yaw acceleration,
/// and performs a SWEPT-SEGMENT collision check against the true
/// cylinders every step (never just the discrete end point).
///
/// Collision truth is used ONLY by the simulator, the safety referee,
/// the 5 Hz privileged expert and the human visualization.  It is never
/// fed back into the 30 Hz planner.

#include "il_2d_multiscale_debug/types.hpp"

namespace il_2d_multiscale_debug {

struct SimStepResult {
    VehicleState2D state;
    bool collision = false;
    bool boundary_collision = false;  // vehicle centre crossed region bounds
    int collision_obstacle_id = -1;
    double collision_penetration = 0.0;  // m (positive when inside surface)
};

class PlanarVehicleSimulator {
public:
    explicit PlanarVehicleSimulator(const Params2D& p) : p_(p) {}

    void reset(const VehicleState2D& s) { state_ = s; }
    const VehicleState2D& state() const { return state_; }

    /// Advance one dt with the commanded body velocity + yaw rate.
    SimStepResult step(double vx_body, double vy_body, double yaw_rate_cmd,
                       const Scene2D& scene);

    void setDt(double dt) { dt_ = dt; }
    double dt() const { return dt_; }

private:
    Params2D p_;
    VehicleState2D state_;
    double dt_ = 1.0 / 30.0;
};

}  // namespace il_2d_multiscale_debug
