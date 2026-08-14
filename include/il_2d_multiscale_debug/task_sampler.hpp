#pragma once
/// @file   task_sampler.hpp
/// @brief  Deterministic start / goal / initial-yaw sampling with CAUSAL
///         task qualification.
///
/// * start and goal are sampled from the MAIN connected component, both
///   satisfying esdf > safety_clearance (strict);
/// * |start - goal| >= min_start_goal_distance;
/// * static A* re-confirms grid connectivity (configurable);
/// * initial yaw = direction(start→goal) + random bias with magnitude in
///   [yaw_bias_min_deg, yaw_bias_max_deg], random sign.
///
/// CAUSAL QUALIFICATION (configurable):
/// If the straight start→goal line is blocked by a major obstacle, LEFT
/// and RIGHT constrained routes are pre-generated around the blocker.
///   * with require_both_sides_feasible=true (recommended) only tasks
///     where BOTH homotopy branches are globally feasible are accepted —
///     this makes the runtime left/right decision observable without ever
///     hitting TASK_INVALID_FOR_CAUSAL_RULE for this cause;
///   * clear (unblocked) and zero-obstacle tasks are always accepted;
///   * if no legal pair passes, sampling continues / fails explicitly.

#include "il_2d_multiscale_debug/connectivity_analyzer.hpp"
#include "il_2d_multiscale_debug/left_right_route_planner.hpp"
#include "il_2d_multiscale_debug/types.hpp"

#include <random>
#include <string>

namespace il_2d_multiscale_debug {

class TaskSampler {
public:
    explicit TaskSampler(const Params2D& p) : p_(p) {}

    /// Sample a task.  Returns false (with reason) if no legal task could
    /// be sampled within the attempt budget.
    bool sampleTask(const Scene2D& scene, const TruthEsdf2D& esdf,
                    const ConnectivityAnalyzer& conn, uint64_t seed,
                    uint64_t task_id, Task2D& out, std::string& reason);

    /// Project an arbitrary world point onto the nearest selectable cell
    /// (esdf > safety_clearance, main component) within a bounded search
    /// radius.  Returns false if none found.
    bool snapToSelectable(const Scene2D& scene, const TruthEsdf2D& esdf,
                          const ConnectivityAnalyzer& conn, const Vec2d& p,
                          Vec2d& out) const;

private:
    bool randomFreeCellInMain(const Scene2D& scene, const TruthEsdf2D& esdf,
                              const ConnectivityAnalyzer& conn, Vec2d& out);

    /// If the straight start→goal corridor is blocked by a major obstacle,
    /// fills `blocker` and returns true.  Uses only true geometry (this is
    /// task qualification, a privileged pre-check — not a 30 Hz input).
    bool findStraightBlocker(const Scene2D& scene, const Vec2d& start,
                             const Vec2d& goal, BlockerInfo& blocker) const;

    /// Causal qualification for a candidate pair.  Returns true when the
    /// pair is acceptable (unblocked, or both required sides feasible).
    bool causalQualify(const Scene2D& scene, const TruthEsdf2D& esdf,
                       const Vec2d& start, const Vec2d& goal) const;

    Params2D p_;
    std::mt19937_64 rng_{42};
};

}  // namespace il_2d_multiscale_debug
