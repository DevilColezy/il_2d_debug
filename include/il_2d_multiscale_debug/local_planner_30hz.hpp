#pragma once
/// @file   local_planner_30hz.hpp
/// @brief  30 Hz local obstacle-avoidance expert.
///
/// INFORMATION BOUNDARY (enforced by the interface):
///   plan(VehicleState2D, LocalObservation, LocalTarget)
///   — the planner CANNOT see the global ESDF, the obstacle truth, the
///     global path or the scene.  It sees only the current state, the
///     local observation (with short-term history) and the current local
///     target (position + update_event, zero-order held by the FSM).
///
/// Behaviour:
///   * TURN_TO_TARGET with hysteresis (default enter > 42°, exit <= 8°
///     only after measured yaw rate also settles);
///   * per-tick candidate search (speed × lateral × yaw-rate samples),
///     forward-integrated under the vehicle's accel limits, validated
///     against FOV, known-FREE cells and observed clearance, scored by
///     progress/soft-clearance/smoothness/control-change/target alignment;
///   * immediate replan when the previously executed trajectory is cut
///     by a newly observed obstacle; emergency brake if the stopping
///     distance is not available;
///   * macro gating: only BLOCKED_BY_OBSERVED_OBSTACLE (a real observed
///     blockage) counts towards the consecutive-failure budget.  UNKNOWN
///     or FOV reasons never trigger the macro expert.

#include "il_2d_multiscale_debug/types.hpp"
#include "il_2d_multiscale_debug/kinematics.hpp"

#include <string>
#include <vector>

namespace il_2d_multiscale_debug {

struct LocalPlannerCandidate {
    // ── Rollout INTENT (long-term desired control, from the YAML samples /
    //    feedback laws).  The 2.5 s forward prediction is made with these. ──
    double desired_vx_body = 0.0;
    double desired_vy_body = 0.0;
    double desired_yaw_rate = 0.0;
    // ── EXECUTABLE OUTPUT: the one-control-period reachable command from
    //    the current state toward the intent.  ONLY this is sent to the
    //    simulator and recorded as the expert label. ──
    double vx_body = 0.0;
    double vy_body = 0.0;
    double yaw_rate = 0.0;

    // Nominal (full-horizon) trajectory predicted with the INTENT.
    Trajectory2D nominal_traj;
    // Certified executable safe-prefix trajectory (what will really be
    // executed before the next 30 Hz replan).
    Trajectory2D traj;
    double cost = std::numeric_limits<double>::infinity();
    double min_clearance = std::numeric_limits<double>::infinity();
    bool feasible = false;
    std::string reject_reason = "";
    // ── Cost parts (for diagnostics + deterministic tie-breaking) ──
    double cost_progress = 0.0;
    double cost_clearance = 0.0;
    double cost_smoothness = 0.0;
    double cost_speed_change = 0.0;
    double cost_yaw_rate_change = 0.0;
    double cost_terminal_heading = 0.0;
    double cost_velocity_alignment = 0.0;
    double cost_cross_track = 0.0;
    // Alignment metrics (rad / m) used for cost and tie-breaking.
    double terminal_heading_error_rad = 0.0;
    double velocity_alignment_error_rad = 0.0;
    double cross_track_error_m = 0.0;
    // Deterministic enumeration index (diagnostic; the final tie-break is
    // the side-neutral command hash, never this index).
    int stable_index = 0;
    // ── Soft-clearance / dynamic-envelope metrics (v4) ──────────────
    // Min observed clearance over the whole swept path (real measured
    // distance to the nearest OCCUPIED cell, inf only when nothing was
    // found inside the soft/dynamic search radius).
    double soft_min_clearance = std::numeric_limits<double>::infinity();
    // Max required dynamic clearance and max closing speed over the path.
    double max_dynamic_required_clearance = 0.0;
    double max_closing_speed = 0.0;
    // Side-neutral deterministic tie-break hash over the intent values.
    uint64_t tie_hash = 0;
    // ── Progress / prefix / category diagnostics (v5) ──────────────
    double safe_prefix_duration_s = 0.0;   // certified executable prefix (s)
    double nominal_progress_m = 0.0;       // progress over the 2.5 s nominal rollout
    double executable_progress_m = 0.0;    // progress over the certified prefix
    double achievable_progress_m = 0.0;    // max_speed * scoring_horizon (normaliser)
    bool progress_qualified = false;       // valid target-direction progress
    bool stationary = false;               // executable output is (near-)zero
    PlannerStatus status = PlannerStatus::NO_SAFE_CANDIDATE;
    // ── Continuous early-avoidance risk (v7) ───────────────────────
    // Normalized obstacle-risk cost term added to `cost` (drives gradual
    // avoidance long before the hard clearance gate).
    double obstacle_risk_cost = 0.0;
    // Closest approach (m) of the nominal rollout to any observed obstacle
    // (inf when none was in the risk field).
    double predicted_closest_clearance =
        std::numeric_limits<double>::infinity();
    // Time (s) to the closest approach of the dominant obstacle.
    double time_to_collision = std::numeric_limits<double>::infinity();
    // Normalized avoidance strength (== obstacle_risk_cost).
    double avoidance_strength = 0.0;
    bool avoidance_active = false;
};

/// Light result of a NON-mutating preview plan (used by the macro exit
/// check: "can the 30 Hz expert independently plan to the goal?").
struct PreviewResult {
    bool success = false;
    bool turn_mode = false;
    bool emergency_brake = false;
    FailureReason failure_reason = FailureReason::NONE;
    // v5: extended outcome so the 5 Hz macro exit requires a SAFE AND
    // PROGRESSING trajectory (a safe stop alone never qualifies).
    PlannerStatus planner_status = PlannerStatus::NO_SAFE_CANDIDATE;
    bool has_progressing_trajectory = false;
    double executable_progress_m = 0.0;
    double safe_prefix_duration_s = 0.0;
    double selected_output_speed_mps = 0.0;
};

class LocalPlanner30Hz {
public:
    explicit LocalPlanner30Hz(const Params2D& p) : p_(p) {}

    /// Full planning step.  Mutates internal hysteresis + current
    /// trajectory bookkeeping.
    PlannerResult plan(const VehicleState2D& state, const LocalObservation& obs,
                       const LocalTarget& target);

    /// Non-mutating preview (macro exit check).  Same algorithm, no
    /// state change.
    PreviewResult previewPlan(const VehicleState2D& state,
                              const LocalObservation& obs,
                              const LocalTarget& target);

    /// True iff the vehicle can come to a full stop (under max accel)
    /// within the observed free space ahead.
    bool canBrakeSafely(const VehicleState2D& state,
                        const LocalObservation& obs) const;

    /// Hysteresis state (exposed for diagnostics).
    bool turnHysteresisActive() const { return turn_hysteresis_active_; }
    void resetTurnHysteresis() { turn_hysteresis_active_ = false; }
    /// Reset all per-task planner memory, including command-change history.
    void reset();

private:
    /// Static macro-handoff base clearance used by this planner:
    ///   handoff_clearance = scene_safety_clearance
    ///                     + macro_route_clearance_margin
    ///                     + clearance_discretization_margin_m
    /// All three inputs are STATIC configuration (no runtime privileged
    /// information), so the 30 Hz planner may use it.
    double handoffClearance() const {
        const double geometric =
            p_.scene_safety_clearance + p_.macro_route_clearance_margin +
            p_.lp_clearance_discretization_margin_m;
        return std::max(p_.lp_nominal_clearance_m, geometric);
    }

    /// Speed-dependent required dynamic clearance for a candidate sample
    /// closing on an observed obstacle:
    ///   closing > 0: required = handoff_clearance + closing_speed*reaction
    ///                          + closing_speed^2 / (2 * max_accel)
    ///   closing <= 0 (separating / recovery): required = HARD clearance.
    /// `closing_speed` is the component of the world velocity along the
    /// unit vector from the sample toward the nearest OCCUPIED cell centre
    /// (clamped at 0 — tangential / separating motion is never penalised).
    /// v7: separating/recovery motion is only required to keep the HARD
    /// surface clearance, so a trajectory that starts in the 0.5..0.65 m
    /// band and climbs out monotonically is accepted (the nominal clearance
    /// is a path-quality reference, not a rejection gate).
    double requiredClearance(double closing_speed) const {
        const double v = std::max(0.0, closing_speed);
        if (v <= 1e-9) return p_.lp_min_clearance;
        return handoffClearance() + v * p_.lp_obstacle_reaction_time_s +
               (v * v) / std::max(1e-6, 2.0 * p_.lp_max_accel);
    }

    /// Search radius for the nearest-OCCUPIED query at a candidate sample:
    /// at least the soft clearance radius, widened when the dynamic
    /// envelope needs a larger query (computed with the TOTAL speed, an
    /// upper bound of the closing speed).
    double clearanceSearchRadius(double total_speed) const {
        const double dyn =
            handoffClearance() +
            total_speed * p_.lp_obstacle_reaction_time_s +
            (total_speed * total_speed) /
                std::max(1e-6, 2.0 * p_.lp_max_accel);
        return std::max(p_.lp_soft_clearance_radius_m, dyn);
    }

    /// One 30 Hz sample of the deterministic stall / control-oscillation
    /// detector window.
    struct LimitCycleSample {
        uint64_t target_update_event = 0;  // diagnostic only
        uint64_t mission_revision = 0;
        double dist_to_target = 0.0;
        double vx_body = 0.0;
        double vy_body = 0.0;
        double yaw_rate = 0.0;
        bool blocked = false;      // observed or dynamic-clearance blockage
        Vec2d position{0.0, 0.0};
        Vec2d target_position{0.0, 0.0};
    };

    /// Deterministic last-resort hash over command values (FNV-1a over the
    /// IEEE bit patterns).  It is evaluated only after geometric and
    /// command-change ties and never consumes a LEFT/RIGHT semantic label.
    static uint64_t commandTieHash(double vx, double vy, double yr);

    /// Update the stall/oscillation window with this tick's sample and
    /// return true when the detector fires (see README §5.4).  Only ever
    /// called with mutate==true.  Clears the window only on a mission
    /// revision or a large target discontinuity;
    /// ordinary target-value update events preserve the window.
    bool updateLimitCycle(const PlannerResult& res, const LocalTarget& target,
                          const VehicleState2D& state);

    PlannerResult computePlan(const VehicleState2D& state,
                              const LocalObservation& obs,
                              const LocalTarget& target, bool mutate);
    bool currentTrajectoryBlocked(const VehicleState2D& state,
                                  const LocalObservation& obs,
                                  bool& dynamic_violation) const;
    std::vector<LocalPlannerCandidate> generateCandidates(
        const VehicleState2D& state) const;
    /// The ONE shared output semantic: project an INTENT command through the
    /// one-control-period reachable acceleration / yaw-acceleration window
    /// (then apply the absolute speed / yaw-rate caps).  Used by EVERY mode
    /// — ordinary lattice candidates, TURN_TO_TARGET, the terminal feedback
    /// controller and the emergency brake — so a turn can never output a
    /// raw 2 rad/s while a lattice candidate is limited to one period.
    BodyCommand2D reachableCommand(const VehicleState2D& state,
                                   const BodyCommand2D& intent) const;
    /// Body-frame velocity of the current state.
    Vec2d bodyVelocity(const VehicleState2D& state) const;
    /// Per-mission bookkeeping (mutate only): detect a mission_revision
    /// change or a > target_discontinuity_reset_m target jump and, when so,
    /// reset planner memory (command continuity + stall window).  Always
    /// updates the last-seen mission/target bookkeeping.  Returns true when
    /// planner memory was reset this tick.
    bool updateMissionState(const LocalTarget& target, PlannerResult& res);
    /// Raw terminal feedback INTENT (no reachability projection).  The
    /// executable output is reachableCommand(state, terminalIntent(...)).
    BodyCommand2D terminalIntent(const VehicleState2D& state,
                                 const LocalTarget& target) const;
    LocalPlannerCandidate makeTerminalCandidate(
        const VehicleState2D& state, const LocalTarget& target) const;
    /// Collect the candidate-local OCCUPIED cells needed by hard/dynamic
    /// clearance validation.
    void collectOccupiedCells(const Trajectory2D& traj,
                              const LocalObservation& obs,
                              std::vector<Vec2d>& out) const;
    /// Collect the wider, spatial early-risk neighbourhood once per planner
    /// tick.  It is shared by every candidate so a 5 m perception horizon
    /// does not cause a full grid scan hundreds of times per tick.
    void collectRiskOccupiedCells(const VehicleState2D& state,
                                  const LocalObservation& obs,
                                  std::vector<Vec2d>& out) const;
    /// Compute the continuous early-avoidance risk of ONE candidate from
    /// its nominal rollout and the pre-collected observed OCCUPIED cells.
    /// Fills obstacle_risk_cost / predicted_closest_clearance /
    /// time_to_collision / avoidance_strength / avoidance_active.  Only
    /// current local observation is used (never privileged data). TTC is
    /// the first predicted entry into the nominal-clearance shell, not the
    /// time at which an arbitrary closest point happens to occur.
    void computeObstacleRisk(LocalPlannerCandidate& c,
                             const VehicleState2D& state,
                             const std::vector<Vec2d>& occ_cells) const;
    /// Assess the local vehicle→LocalTarget corridor against the current
    /// local observation: sets `blocked` and the longitudinal distance (m)
    /// to the first obstacle that intersects the corridor (NaN when clear).
    void assessLocalCorridor(const VehicleState2D& state,
                             const LocalObservation& obs,
                             const LocalTarget& target, bool& blocked,
                             double& first_blocking_distance_m) const;
    /// Evaluate one candidate.  Validates the swept path and computes the
    /// EXECUTABLE SAFE PREFIX (points that stay inside the current FOV and
    /// known-FREE space). FOV/UNKNOWN boundaries may truncate a sufficiently
    /// long progressing prefix. An observed hard/dynamic-clearance violation
    /// rejects the whole candidate for every LocalTarget source; there is no
    /// macro-guide exception in the 30 Hz layer. On rejection the
    /// FIRST decisive reason is returned in both `reject_reason` (string,
    /// for the message) and `reject_enum` (typed, for per-reason counting).
    bool evaluateCandidate(LocalPlannerCandidate& c, const VehicleState2D& state,
                           const LocalObservation& obs, const LocalTarget& target,
                           const std::vector<Vec2d>& risk_occ_cells,
                           std::string& reject_reason,
                           CandidateRejectReason& reject_enum) const;
    bool corridorBlockedByObserved(const VehicleState2D& state,
                                   const LocalObservation& obs,
                                   const LocalTarget& target) const;
    double stoppingDistance(const VehicleState2D& state) const;
    bool spaceToStop(const VehicleState2D& state, const LocalObservation& obs,
                     double dist) const;

    Params2D p_;
    bool turn_hysteresis_active_ = false;
    Trajectory2D current_trajectory_;  // last accepted trajectory (blockage check)
    BodyCommand2D last_command_;       // last EXECUTABLE output command
    bool has_last_command_ = false;
    // ── Per-mission continuity tracking (v5) ───────────────────────
    // Smoothing / jerk continuity and the stall window persist across small
    // continuous LocalTarget moves; they are reset ONLY by a mission
    // revision change, a target discontinuity reset, or reset(). A plain
    // update_event change never resets anything.
    uint64_t last_mission_revision_ = 0;
    Vec2d last_target_position_{0.0, 0.0};
    bool last_target_valid_ = false;
    // Deterministic stall / control-oscillation detector state.  Only
    // mutated by updateLimitCycle() (mutate==true); previewPlan never
    // touches it.  reset() clears it.
    std::vector<LimitCycleSample> limit_cycle_window_;
    bool limit_cycle_detected_ = false;
    uint64_t last_cycle_mission_revision_ = 0;
};

}  // namespace il_2d_multiscale_debug
