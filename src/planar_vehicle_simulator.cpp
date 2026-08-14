#include "il_2d_multiscale_debug/planar_vehicle_simulator.hpp"

#include "il_2d_multiscale_debug/kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace il_2d_multiscale_debug {

namespace {
/// Min distance from segment a→b to point p (returns signed: negative if
/// the perpendicular projection falls outside the segment and the nearest
/// endpoint is used).
double segmentPointDistance(const Vec2d& a, const Vec2d& b, const Vec2d& p) {
    const Vec2d ab = b - a;
    const double len2 = ab.squaredNorm();
    if (len2 < 1e-12) return (p - a).norm();
    const double t = clamp(((p - a).dot(ab)) / len2, 0.0, 1.0);
    const Vec2d proj = a + ab * t;
    return (p - proj).norm();
}
}  // namespace

SimStepResult PlanarVehicleSimulator::step(double vx_body, double vy_body,
                                           double yaw_rate_cmd,
                                           const Scene2D& scene) {
    SimStepResult res;
    res.state = state_;
    const double dt = dt_;

    // ── Same shared kinematics as the 30 Hz planner's prediction. ─────
    const VehicleState2D next =
        integrateKinematicStep(state_, BodyCommand2D{vx_body, vy_body, yaw_rate_cmd},
                               dt, p_);

    // ── Swept-segment collision against true cylinders. ──────────────
    // (Truth used ONLY by the simulator / referee / 5 Hz expert / human.)
    const Vec2d pos_old = state_.position;
    const Vec2d pos_new = next.position;
    double best_pen = -std::numeric_limits<double>::infinity();
    int best_id = -1;
    for (const auto& o : scene.obstacles) {
        const double d = segmentPointDistance(pos_old, pos_new, o.center);
        const double pen = o.radius + p_.drone_radius - d;
        if (pen > best_pen) {
            best_pen = pen;
            best_id = o.id;
        }
    }

    // ── Swept check against the region boundary (drone radius included);
    //    the vehicle centre may never cross the valid region bounds. ──
    bool boundary = false;
    if (pos_new.x() - p_.drone_radius < scene.min_bounds.x() ||
        pos_new.x() + p_.drone_radius > scene.max_bounds.x() ||
        pos_new.y() - p_.drone_radius < scene.min_bounds.y() ||
        pos_new.y() + p_.drone_radius > scene.max_bounds.y()) {
        boundary = true;
    }

    if (best_pen > 0.0 || boundary) {
        res.collision = true;
        res.boundary_collision = boundary;
        res.collision_obstacle_id = best_id;
        res.collision_penetration = std::max(0.0, best_pen);
        // Stop the vehicle at the impact pose.
        res.state.position = pos_new;
        res.state.yaw = next.yaw;
        res.state.velocity_world = Vec2d::Zero();
        res.state.yaw_rate = 0.0;
        state_ = res.state;
        return res;
    }

    res.state = next;
    state_ = next;
    return res;
}

}  // namespace il_2d_multiscale_debug
