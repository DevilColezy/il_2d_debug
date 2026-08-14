#pragma once
/// @file   fov_raycaster_2d.hpp
/// @brief  Synthesizes the instantaneous FOV observation with 2D ray
///         casting against true cylinder surfaces.
///
/// Produces a THREE-STATE patch (FREE / OCCUPIED / UNKNOWN):
///   * only cells inside the current FOV wedge and within range are
///     considered;
///   * rays stop at the first obstacle surface → the surface ring is
///     OCCUPIED and everything behind it stays UNKNOWN;
///   * cells outside the FOV stay UNKNOWN;
///   * the true circle is NEVER filled wholesale into the local map —
///     the observation is built purely by ray-surface intersections, so
///     occluded geometry is genuinely unknown to the planner.
///
/// GRID ALIGNMENT (root-cause contract): the returned patch shares the
/// EXACT global grid of ObservedGrid2D — its origin is snapped to a
/// global grid boundary derived from scene.min_bounds + n*resolution
/// (never `state.position - range`).  Consequently every patch cell
/// centre maps (via the shared worldToGrid / floor convention) to the
/// SAME global cell in the history map.  The patch covers at least the
/// vehicle-centred disc of radius obs_range_m, and the vehicle's own
/// cell is explicitly marked FREE (sensor-origin semantics; collision is
/// still owned by the truth referee).
///
/// The returned patch has no history; ObservedGrid2D merges it into the
/// persistent short-term local map.  Truth is used here ONLY to simulate
/// the sensor measurement process — this is the observation synthesis
/// stage, not a privilege channel into the 30 Hz planner.

#include "il_2d_multiscale_debug/types.hpp"

namespace il_2d_multiscale_debug {

class FovRaycaster2D {
public:
    explicit FovRaycaster2D(const Params2D& p) : p_(p) {}

    /// Cast a patch of size (2*range)x(2*range) centred on the vehicle.
    LocalObservation cast(const VehicleState2D& state, const Scene2D& scene,
                          uint64_t tick) const;

private:
    Params2D p_;
};

}  // namespace il_2d_multiscale_debug
