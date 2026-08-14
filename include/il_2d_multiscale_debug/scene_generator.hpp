#pragma once
/// @file   scene_generator.hpp
/// @brief  Deterministic 2D scene generator.
///
/// Obstacles are 2D circles (horizontal projection of infinitely tall
/// cylinders).  Every placement must satisfy:
///   * surface gap between any two obstacles
///         >= 2*safety_clearance + passage_margin
///   * distance from the obstacle surface to every region boundary
///         >= boundary_margin + safety_clearance
/// If a single obstacle cannot be placed within max_attempts_per_obstacle
/// rejection samples, the WHOLE scene is resampled (never shrink radii,
/// never reduce clearance, never accept overlap).  All randomness comes
/// from one explicit uint64_t seed.
///
/// A legal EMPTY scene is produced ONLY when the random draw asks for
/// zero obstacles.  If a non-zero obstacle count cannot be placed, the
/// generator reports an explicit FAILURE (SceneGenerationResult) so the
/// caller can switch seeds or re-request — it never silently degrades.

#include "il_2d_multiscale_debug/types.hpp"

#include <random>
#include <string>
#include <vector>

namespace il_2d_multiscale_debug {

/// Result of a scene generation attempt (explicit success/failure).
struct SceneGenerationResult {
    bool success = false;
    Scene2D scene;
    std::string reason;
    int requested_obstacle_count = 0;
    int placed_obstacle_count = 0;
};

class SceneGenerator {
public:
    explicit SceneGenerator(const Params2D& p) : p_(p) {}

    /// Deterministically generate a scene from `seed`.  scene_id is
    /// caller-provided (a counter), seed drives all randomness.  Never
    /// silently returns a degraded scene: failure is explicit.
    SceneGenerationResult generate(uint64_t seed, uint64_t scene_id);

private:
    bool placeOne(const std::vector<Obstacle2D>& existing, Obstacle2D& out);
    bool validPlacement(const Vec2d& c, double r,
                        const std::vector<Obstacle2D>& existing) const;
    double sampleRadius();
    Vec2d sampleCenter(double r);
    int randInt(int lo, int hi);
    double rand01();

    Params2D p_;
    std::mt19937_64 rng_{42};
};

}  // namespace il_2d_multiscale_debug
