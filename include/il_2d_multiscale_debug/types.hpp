#pragma once
/// @file   types.hpp
/// @brief  Shared data types and parameters for il_2d_multiscale_debug.
///
/// This package is a SELF-CONTAINED, pure-2D debug harness.  It has no
/// dependency on il_dataset, il_multiscale_dataset, Flightmare or Unity.
///
/// ── 2D frame & yaw convention (documented, self-consistent) ─────────
///   World frame: X → right, Y → up (standard 2D math frame).
///   Yaw is CCW-positive, vehicle forward direction = (cos yaw, sin yaw).
///   Body frame: +X forward (nose), +Y left (90° CCW from nose).
///   A target with positive bearing (atan2(dy,dx) - yaw) is on the LEFT.
///   (This differs from the Flightmare FLU convention on purpose: the
///   debug harness never talks to Flightmare, and a standard 2D frame
///   keeps the math unambiguous.)

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace il_2d_multiscale_debug {

using Vec2d = Eigen::Vector2d;

// ═══════════════════════════════════════════════════════════════════
//  Small math helpers
// ═══════════════════════════════════════════════════════════════════
inline double deg2rad(double d) { return d * M_PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / M_PI; }
inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}
/// Wrap an angle into [-pi, pi].
inline double wrapAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}
/// 2D rotation of v by CCW angle a.
inline Vec2d rot2(const Vec2d& v, double a) {
    const double c = std::cos(a), s = std::sin(a);
    return Vec2d(c * v.x() - s * v.y(), s * v.x() + c * v.y());
}
/// Signed 2D cross product z = a.x*b.y - a.y*b.x.
inline double cross2(const Vec2d& a, const Vec2d& b) {
    return a.x() * b.y() - a.y() * b.x();
}

// ═══════════════════════════════════════════════════════════════════
//  WORLD ↔ GLOBAL-GRID index convention (ONE package-wide convention)
//  ═══════════════════════════════════════════════════════════════════
//  Every world-aligned grid (the instantaneous FOV patch AND the merged
//  history map) is anchored at the SAME global grid whose origin is
//  `min_bounds + n*resolution` (grid-aligned).  The unique index rule is:
//      ix = floor((world - min_bounds) / resolution)
//      cell_centre = min_bounds + (ix + 0.5) * resolution
//  floor() handles negative coordinates correctly, and an exact grid
//  boundary is never ambiguous (floor of an integer is that integer).
//  All consumers MUST use these helpers — never re-derive a different
//  formula — so a patch cell and the history cell it merges into are
//  ALWAYS the same global cell.
struct GridIndex2D {
    int ix = 0;
    int iy = 0;
};

/// floor((world - min_bounds) / resolution) for a grid anchored at
/// `min_bounds`.  `min_bounds` may be any grid-aligned origin (e.g. the
/// global scene min_bounds, or a patch origin which is itself aligned).
inline GridIndex2D worldToGrid(const Vec2d& world, const Vec2d& min_bounds,
                               double resolution) {
    const double inv = 1.0 / resolution;
    return {static_cast<int>(std::floor((world.x() - min_bounds.x()) * inv)),
            static_cast<int>(std::floor((world.y() - min_bounds.y()) * inv))};
}

/// World position of a cell centre on a grid anchored at `min_bounds`.
inline Vec2d gridCellCenter(int ix, int iy, const Vec2d& min_bounds,
                            double resolution) {
    return Vec2d(min_bounds.x() + (static_cast<double>(ix) + 0.5) * resolution,
                 min_bounds.y() + (static_cast<double>(iy) + 0.5) * resolution);
}

// ═══════════════════════════════════════════════════════════════════
//  Enum: candidate rejection reasons (first decisive reason only)
// ═══════════════════════════════════════════════════════════════════
enum class CandidateRejectReason : uint8_t {
    NONE = 0,
    NOT_KNOWN_FREE = 1,
    OUTSIDE_CURRENT_FOV = 2,
    OBSERVED_CLEARANCE_TOO_SMALL = 3,
    NO_PROGRESS = 4,
    OTHER = 5,
    // The candidate's swept path violates the speed-dependent braking /
    // macro-handoff dynamic clearance envelope (observed OCCUPIED cell
    // closer than required_clearance while closing on it).  Like
    // OBSERVED_CLEARANCE_TOO_SMALL this rejects the WHOLE candidate (it is
    // a real observed blockage, never a prefix-truncatable FOV/UNKNOWN
    // boundary) and counts toward the real-blockage macro budget.
    INSUFFICIENT_BRAKING_CLEARANCE = 6,
};

inline const char* candidateRejectReasonName(CandidateRejectReason r) {
    switch (r) {
        case CandidateRejectReason::NONE: return "NONE";
        case CandidateRejectReason::NOT_KNOWN_FREE: return "NOT_KNOWN_FREE";
        case CandidateRejectReason::OUTSIDE_CURRENT_FOV: return "OUTSIDE_CURRENT_FOV";
        case CandidateRejectReason::OBSERVED_CLEARANCE_TOO_SMALL:
            return "OBSERVED_CLEARANCE_TOO_SMALL";
        case CandidateRejectReason::NO_PROGRESS: return "NO_PROGRESS";
        case CandidateRejectReason::OTHER: return "OTHER";
        case CandidateRejectReason::INSUFFICIENT_BRAKING_CLEARANCE:
            return "INSUFFICIENT_BRAKING_CLEARANCE";
    }
    return "UNKNOWN";
}

// ═══════════════════════════════════════════════════════════════════
//  Enum: cell state of the local observation
// ═══════════════════════════════════════════════════════════════════
enum class CellState : uint8_t { FREE = 0, OCCUPIED = 1, UNKNOWN = 2 };

// ═══════════════════════════════════════════════════════════════════
//  Enum: FSM states
//
//  v9: the 5 Hz expert is no longer a "macro route planner" with its own
//  guidance states.  It is a LOCAL-OBSERVABILITY JUDGE + TARGET
//  CORRECTOR that runs on every 5 Hz boundary and outputs a
//  TargetCorrectionDirective (PASS_THROUGH / NORMAL_CORRECTION /
//  TURN_LEFT / TURN_RIGHT), converted every 30 Hz tick by the
//  EffectiveTargetAdapter into the LocalTarget the 30 Hz planner sees.
//  The FSM therefore only owns the 30 Hz planner's own behaviour states
//  (DIRECT_LOCAL / TURN_TO_TARGET) plus the terminal states.  The old
//  LOCAL_BLOCKED_PENDING / MACRO_SELECT_SIDE / MACRO_GUIDANCE /
//  MACRO_EXIT_PENDING states are REMOVED (the 5 Hz corrector never reads
//  the 30 Hz outcome, so there is nothing to "block" on).
enum class FsmState : uint8_t {
    DIRECT_LOCAL = 0,
    TURN_TO_TARGET = 1,
    GOAL_REACHED = 2,
    TASK_INVALID = 3,
    COLLISION = 4,
    TIMEOUT = 5,
};

inline const char* fsmStateName(FsmState s) {
    switch (s) {
        case FsmState::DIRECT_LOCAL: return "DIRECT_LOCAL";
        case FsmState::TURN_TO_TARGET: return "TURN_TO_TARGET";
        case FsmState::GOAL_REACHED: return "GOAL_REACHED";
        case FsmState::TASK_INVALID: return "TASK_INVALID";
        case FsmState::COLLISION: return "COLLISION";
        case FsmState::TIMEOUT: return "TIMEOUT";
    }
    return "UNKNOWN";
}

// ═══════════════════════════════════════════════════════════════════
//  Enum: local planner failure reasons (macro gating depends on this)
// ═══════════════════════════════════════════════════════════════════
enum class FailureReason : uint8_t {
    NONE = 0,
    TARGET_OUTSIDE_FOV = 1,     // handled by TURN_TO_TARGET, not macro
    NO_SAFE_CANDIDATE = 2,      // sample-set / UNKNOWN-driven, not macro
    BLOCKED_BY_OBSERVED_OBSTACLE = 3,  // real observed blockage → macro eligible
    // Safe candidates exist but NONE has valid target-direction progress
    // AND there is no observed-blockage evidence.  This is NEVER a success
    // and NEVER macro-eligible: an open-space success+stop is a 30 Hz
    // internal regression that must be fixed inside the planner, not
    // masked by the 5 Hz privileged expert.
    STALLED_WITHOUT_PROGRESS = 4,
};

inline const char* failureReasonName(FailureReason r) {
    switch (r) {
        case FailureReason::NONE: return "NONE";
        case FailureReason::TARGET_OUTSIDE_FOV: return "TARGET_OUTSIDE_FOV";
        case FailureReason::NO_SAFE_CANDIDATE: return "NO_SAFE_CANDIDATE";
        case FailureReason::BLOCKED_BY_OBSERVED_OBSTACLE: return "BLOCKED_BY_OBSERVED_OBSTACLE";
        case FailureReason::STALLED_WITHOUT_PROGRESS: return "STALLED_WITHOUT_PROGRESS";
    }
    return "UNKNOWN";
}

// ═══════════════════════════════════════════════════════════════════
//  Enum: planner candidate-category / outcome (v5)
// ═══════════════════════════════════════════════════════════════════
//  The selection is HIERARCHICAL (hard safety → valid progress → dynamic
//  clearance + executable prefix → in-category costs), not a pure weighted
//  sum.  Away from the goal a SAFE_PROGRESSING candidate always beats a
//  SAFE_HOLD/stop candidate; zero-speed commands are only selected in the
//  terminal settling region, in TURN_TO_TARGET, during an emergency brake,
//  or when every motion candidate is unsafe.
enum class PlannerStatus : uint8_t {
    SAFE_PROGRESSING = 0,             // safe AND valid target-direction progress
    SAFE_HOLD = 1,                    // safe zero-progress hold / settled distance-1 turn
    TERMINAL_SETTLING = 2,            // inside the terminal convergence region
    TURNING = 3,                      // TURN_TO_TARGET (rotate in place)
    EMERGENCY_BRAKE = 4,              // stopping distance not available
    BLOCKED_BY_OBSERVED_OBSTACLE = 5, // no safe candidate, real observed blockage
    NO_SAFE_CANDIDATE = 6,            // no safe candidate, sample-set/UNKNOWN driven
    STALLED_WITHOUT_PROGRESS = 7,     // safe candidates exist, none progresses
    NO_TARGET = 8,                    // no valid LocalTarget
};

inline const char* plannerStatusName(PlannerStatus s) {
    switch (s) {
        case PlannerStatus::SAFE_PROGRESSING: return "SAFE_PROGRESSING";
        case PlannerStatus::SAFE_HOLD: return "SAFE_HOLD";
        case PlannerStatus::TERMINAL_SETTLING: return "TERMINAL_SETTLING";
        case PlannerStatus::TURNING: return "TURNING";
        case PlannerStatus::EMERGENCY_BRAKE: return "EMERGENCY_BRAKE";
        case PlannerStatus::BLOCKED_BY_OBSERVED_OBSTACLE: return "BLOCKED_BY_OBSERVED_OBSTACLE";
        case PlannerStatus::NO_SAFE_CANDIDATE: return "NO_SAFE_CANDIDATE";
        case PlannerStatus::STALLED_WITHOUT_PROGRESS: return "STALLED_WITHOUT_PROGRESS";
        case PlannerStatus::NO_TARGET: return "NO_TARGET";
    }
    return "UNKNOWN";
}

// ═══════════════════════════════════════════════════════════════════
//  Enum: left/right side selection
// ═══════════════════════════════════════════════════════════════════
enum class SideSelection : uint8_t { NONE = 0, LEFT = 1, RIGHT = 2 };

inline const char* sideName(SideSelection s) {
    switch (s) {
        case SideSelection::NONE: return "NONE";
        case SideSelection::LEFT: return "LEFT";
        case SideSelection::RIGHT: return "RIGHT";
    }
    return "UNKNOWN";
}

// ═══════════════════════════════════════════════════════════════════
//  Parameters (mirrors config/default.yaml)
//  All values are read from ROS params by the nodes; the defaults here
//  match the shipped YAML.
// ═══════════════════════════════════════════════════════════════════
struct Params2D {
    // region
    double region_min_x = -20.0, region_max_x = 20.0;
    double region_min_y = -20.0, region_max_y = 20.0;
    double esdf_resolution = 0.1;
    double drone_radius = 0.15;

    // scene generation
    int    scene_min_obstacles = 0, scene_max_obstacles = 12;
    double scene_min_radius = 0.1, scene_max_radius = 8.0;
    std::string scene_radius_distribution = "log_uniform";
    double scene_safety_clearance = 0.5;
    double scene_passage_margin = 0.2;
    double scene_boundary_margin = 0.5;
    int    scene_max_attempts_per_obstacle = 150;
    int    scene_max_total_scene_attempts = 64;

    // connectivity
    int    conn_neighbor = 8;
    double conn_min_main_component_area_m2 = 100.0;

    // task sampling
    double task_min_start_goal_distance = 10.0;
    double task_yaw_bias_min_deg = 20.0;
    double task_yaw_bias_max_deg = 160.0;
    double task_goal_tolerance = 0.4;
    // <= 0 disables the episode timeout; collision/goal/task-invalid remain
    // terminal conditions.
    double task_episode_timeout_s = 0.0;
    bool   task_astar_confirm = true;
    int    task_goal_snap_max_radius_cells = 80;
    int    task_max_sampling_attempts = 400;
    // causal task qualification
    bool   task_require_both_sides_feasible = true;
    int    task_max_blocked_qual_attempts = 80;

    // observation
    double obs_fov_deg = 90.0;
    double obs_range_m = 6.0;
    double obs_resolution = 0.1;
    double obs_ray_angular_res_deg = 0.5;
    uint32_t obs_history_max_age_ticks = 120;

    // local planner
    double lp_horizon_s = 2.5;
    double lp_dt = 0.1;
    std::vector<double> lp_speed_samples{0.0, 0.3, 0.6, 1.2, 1.8, 2.5};
    std::vector<double> lp_lateral_ratio_samples{
        -0.5, -0.3, -0.15, -0.05, 0.0, 0.05, 0.15, 0.3, 0.5};
    // Coarse avoidance turns plus fine residual-heading corrections.
    std::vector<double> lp_yaw_rate_samples{
        -2.0, -1.0, -0.5, -0.25, -0.15, 0.0,
         0.15, 0.25, 0.5, 1.0, 2.0};
    double lp_max_speed = 3.0;
    double lp_max_accel = 2.0;
    double lp_max_yaw_rate = 2.0;
    double lp_max_yaw_accel = 4.0;
    double lp_min_clearance = 0.5;
    // Soft (continuous) clearance-cost radius: observed OCCUPIED cells
    // within this radius produce a smoothly increasing clearance cost
    // (1 near lp_min_clearance, ~0 near the soft radius).  This gives the
    // 30 Hz planner an early avoidance gradient instead of the old
    // all-or-nothing 0.5 m gate.
    double lp_soft_clearance_radius_m = 2.0;
    // Extra clearance reserved for 0.1 m grid discretisation error in the
    // macro-handoff dynamic envelope (handoff_clearance =
    // scene_safety_clearance + macro_route_clearance_margin +
    // clearance_discretization_margin_m).
    double lp_clearance_discretization_margin_m = 0.05;
    // Reaction time (s) used by the dynamic braking/handoff envelope:
    // required_clearance = handoff_clearance + closing_speed * reaction
    //                       + closing_speed^2 / (2 * max_accel)
    double lp_obstacle_reaction_time_s = 0.20;
    // 30 Hz control period (s) used to build the reachable dynamic window
    // for candidate commands: raw samples are clamped to the current
    // velocity ± max_accel*control_period (and yaw-rate window).
    double lp_control_period_s = 0.0333333333;
    // Max allowed target regression (m) for a candidate relative to the
    // current distance to the LocalTarget.  Small zero-progress candidates
    // are allowed (avoid stop/lateral chattering), but candidates clearly
    // moving away from the target are still rejected.
    double lp_max_allowed_regress_m = 0.05;
    // Stall / control-oscillation detector window (30 Hz ticks) and
    // thresholds.  All are local-planner-only state (never privileged).
    int    lp_limit_cycle_window_ticks = 15;
    double lp_limit_cycle_net_progress_m = 0.10;
    // Minimum number of ticks inside the window carrying real observed /
    // dynamic-clearance blockage evidence for condition A.
    int    lp_limit_cycle_min_blocked_ticks = 8;
    // Number of lateral-command sign reversals inside the window (with a
    // significant |vy|) for condition B.
    int    lp_limit_cycle_lateral_flip_count = 2;
    double lp_turn_enter_deg = 42.0;
    double lp_turn_exit_deg = 8.0;
    // TURN_TO_TARGET also requires |state.yaw_rate| <= this before exiting,
    // so a large residual rotation cannot overshoot the target heading.
    double lp_turn_exit_max_yaw_rate = 0.15;
    double lp_turn_k = 2.5;
    // Near-goal heading relaxation: within this distance of the target the
    // TURN_TO_TARGET enter threshold is widened to near_goal_turn_enter_deg
    // (the vehicle decelerates and converges instead of re-spinning near the
    // goal); a target clearly behind (> threshold) still forces a turn.
    double lp_near_goal_heading_relax_distance = 1.0;
    double lp_near_goal_turn_enter_deg = 75.0;
    // Final-goal feedback controller.  Inside this distance the 30 Hz
    // expert first tries a target-directed, continuously decelerating
    // command.  It is accepted only after the same FOV/known-free/clearance
    // validation as every sampled candidate; otherwise normal avoidance is
    // still used.
    double lp_terminal_control_distance = 1.2;
    double lp_terminal_speed_gain = 1.0;
    double lp_terminal_max_speed = 0.6;
    double lp_terminal_max_yaw_rate = 0.5;
    double lp_min_progress_m = 0.05;
    // Minimum executable-output speed (m/s) for a candidate to count as
    // "progressing" (SAFE_PROGRESSING).  The one-control-period reachable
    // first command from standstill is ~max_accel*control_period ≈ 0.067
    // m/s, so this must stay below that to accept a normal accelerating
    // command while still rejecting true stops.  See README §5.5.
    double lp_min_progress_speed_mps = 0.03;
    // Minimum executable progress (m) over the certified safe prefix toward
    // the LocalTarget for a candidate to count as progressing.
    double lp_min_progress_epsilon_m = 0.01;
    // A LocalTarget position jump larger than this (m) is a discontinuity:
    // the 30 Hz planner resets its per-mission memory (command continuity +
    // stall/oscillation window) and reports target_discontinuity_reset.
    // Small continuous moves (e.g. a rolling 5 Hz target network) never
    // reset.  See README §7.
    double lp_target_discontinuity_reset_m = 1.5;
    // Nominal / desired clearance (m).  Used ONLY as a path-quality
    // reference (soft-clearance cost scale / documented handoff base) and
    // NEVER as the "when does avoidance start" trigger distance — the hard
    // safety gate is lp_min_clearance and the early-avoidance trigger is
    // the continuous obstacle-risk cost below.  README §5.7.
    double lp_nominal_clearance_m = 0.65;
    // ── Continuous early-avoidance risk (v7) ───────────────────────
    // A normalized, continuous penalty added to every candidate's total
    // cost.  It becomes non-zero as soon as an observed obstacle is
    // confirmed to lie ahead in the candidate's travel corridor, well
    // before the hard clearance gate, so the planner eases off (decel /
    // small lateral / small turn) gradually instead of only reacting when
    // the surface clearance drops below ~1 m.  README §5.6.
    // Half-width (m) of the local corridor (vehicle → LocalTarget) used to
    // gate the risk: obstacles inside the corridor (or near the predicted
    // trajectory sweep) contribute; off-corridor, off-path obstacles get
    // ~zero risk (no pointless detours).
    double lp_risk_corridor_half_width = 1.0;
    // Longitudinal distance horizon (m) along the candidate direction over
    // which the geometric distance factor ramps from ~0 at the horizon to 1
    // at the hard clearance.
    double lp_risk_distance_horizon_m = 5.0;
    // Time-to-collision horizon (s). TTC is the first predicted entry into
    // the nominal-clearance shell; braking/diverting candidates which never
    // enter it have infinite TTC.
    double lp_risk_ttc_horizon_s = 2.5;
    // Radius (m) around the predicted trajectory sweep within which an
    // observed obstacle counts as "near the path" even when off-corridor.
    double lp_risk_trajectory_radius_m = 1.0;
    // Risk value above which avoidance_active is reported.
    double lp_avoidance_active_threshold = 0.10;
    double lp_brake_stop_margin_m = 0.3;
    // Minimum executable SAFE-PREFIX duration (s) for a candidate: the
    // trajectory is accepted (and truncated to) its prefix that stays
    // inside the current FOV and known-FREE space.  A candidate whose
    // safe prefix is shorter than this is rejected.  Must be >= lp_dt.
    double lp_min_executable_prefix_s = 0.2;
    // Prefixes shorter than this common scoring horizon receive a
    // proportional progress-cost penalty.  This prevents a 0.2 s prefix
    // from beating a fully observed trajectory merely because its
    // alignment/jerk have barely had time to develop.
    double lp_scoring_horizon_s = 0.8;
    // Best-candidate tie tolerance: candidates whose cost differs by less
    // than this are considered tied and ranked by the deterministic
    // secondary rule (heading error, cross-track, |vy|, |yaw_rate|,
    // control change, stable index) — never by enumeration order alone.
    double lp_cost_tie_tolerance = 1e-6;
    // Normalization scale (m) for the cross-track cost term.
    double lp_cross_track_normalize_m = 2.0;
    double cost_w_progress = 1.0;
    double cost_w_clearance = 2.0;
    double cost_w_smoothness = 0.5;
    double cost_w_speed_change = 0.3;
    // Yaw-rate CHANGE cost (command vs current yaw rate + in-trajectory
    // yaw-rate oscillation) — replaces the old absolute-turning cost that
    // penalised legitimate heading correction towards the target.
    double cost_w_yaw_rate_change = 0.3;
    // Keep a preference for looking toward the local target, but do not let
    // nose alignment dominate a safe lateral avoidance manoeuvre.  Velocity
    // alignment remains stronger because it represents actual goal progress.
    double cost_w_terminal_heading = 1.0;
    double cost_w_velocity_alignment = 1.2;
    double cost_w_cross_track = 0.8;
    // Weight of the normalized continuous obstacle-risk cost term (v7).
    double cost_w_obstacle_risk = 3.0;

    // macro expert
    double macro_local_failure_duration_s = 0.4;
    double macro_route_lookahead_min = 2.5;
    double macro_route_lookahead_max = 4.0;
    // ── Macro guide point selection (v7) ───────────────────────────
    // Minimum distance (m) from the vehicle to a macro guide point.  The
    // 5 Hz expert never hands the 30 Hz planner a foot-level target
    // (0.2-0.3 m) that provides no directionality; the guide is advanced
    // along the route until at least this far ahead (and normally sits at
    // the adaptive arc-length lookahead below).
    double macro_guide_min_distance_m = 1.5;
    // Adaptive lookahead: base arc lookahead grows by speed*time_gap (s)
    // up to route_lookahead_max, and shrinks toward route_lookahead_min on
    // curved route sections.
    double macro_guide_lookahead_time_gap_s = 1.0;
    // World-space hysteresis (m): if the newly selected guide is within
    // this distance of the previous guide, keep the previous one so a
    // rolling route never makes the 5 Hz target jitter left/right.
    double macro_guide_hysteresis_m = 0.3;
    // FOV margin (deg) used when clamping the guide into the local FOV.
    double macro_guide_fov_margin_deg = 10.0;
    // Translational displacement (m) per 5 Hz interval below which the FSM
    // counts a macro no-progress sample.
    double macro_no_progress_threshold_m = 0.05;
    // Continuous no-progress duration (s) in macro mode after which the FSM
    // forces a guide re-advance / route refresh (diagnostic + safety).
    double macro_no_progress_duration_threshold_s = 1.0;
    double macro_side_evidence_margin = 0.5;
    double macro_evidence_ray_step_deg = 1.0;
    int    macro_min_evidence_ray_pairs = 4;
    // Effective local-target events follow the VALUE-CHANGE rule: a new
    // event is issued only when the delivered target moved by more than
    // this tolerance (m).
    double macro_local_target_event_tolerance_m = 0.05;
    double macro_exit_fov_margin_deg = 10.0;
    double macro_corridor_half_width = 1.5;
    double macro_corridor_rear_tolerance_m = 0.5;
    double macro_blocking_lateral_span_ratio = 0.5;
    double macro_blocker_clear_dist_m = 1.0;
    // Max lateral distance (m) from the FIXED entry reference route for a
    // vehicle projection to be trusted for the blocker-passed check;
    // beyond this the monotonic progress is held (no false jump to the
    // route end).  Conservative value consistent with the local route
    // corridor width (corridor_half_width=1.5 + margins).
    double macro_blocker_projection_max_dist_m = 2.0;
    // Tolerance (m) on top of max_speed * 5 Hz interval for the per-tick
    // forward progress window of the fixed-route tracking.
    double macro_progress_forward_tolerance_m = 0.5;
    // Backward arc-length window (m) allowed when re-projecting the
    // vehicle onto the fixed entry route (stops at stops).
    double macro_progress_back_window_m = 1.0;
    // Blocker association: min supported visible cells for a MATCHED
    // primary cylinder, and the ratio below which the second-best
    // cylinder makes the match AMBIGUOUS.
    int    macro_blocker_match_min_cells = 3;
    double macro_blocker_match_ambiguity_ratio = 0.5;
    // Max geometric residual (m) for a visible cell to count as support
    // for a truth cylinder (surface/inside distance).
    double macro_blocker_match_surface_tol_m = 0.3;
    int    macro_reentry_guard_ticks = 30;
    double macro_route_clearance_margin = 0.1;
    // Bounded radius (m) of the start-clearance recovery search: when the
    // macro A* start cell falls short of the route clearance (0.1 m
    // discretisation error) while the continuous start is still above
    // scene_safety_clearance, the nearest route-clear cell within this
    // radius is used and a short recovery connection segment (satisfying
    // only scene_safety_clearance) is prepended.  Finite radius / finite
    // node count by construction.
    double macro_start_recovery_max_radius_m = 0.5;
    double macro_route_side_bias = 0.4;
    double macro_homotopy_side_tolerance_m = 0.05;
    double macro_gateway_projection_radius_m = 0.8;
    // Consecutive NO_SAFE_CANDIDATE (UNKNOWN-driven, NOT blocked by an
    // observed obstacle) 30 Hz ticks after which the LOCAL_UNKNOWN_RECOVERY
    // diagnostic becomes active.  NO_SAFE_CANDIDATE never triggers the 5 Hz
    // macro expert and never counts as real blockage.
    int    macro_unknown_recovery_threshold_ticks = 60;

    // ── Local-causal 5 Hz macro expert (v8) ────────────────────────
    // v8: the 5 Hz macro expert is a PURE LOCAL observer.  It never reads
    // the global ESDF, the scene obstacles, global A* routes or global
    // connectivity; every runtime decision (blocker, side, guide, route,
    // blocker-passed, exit) is computed from the current FOV patch + the
    // merged local history map + the vehicle state + the final goal + its
    // own long-term memory (local blocker track / locked side / previous
    // guide).  The parameters below control that local pipeline.
    // Required goal-distance reduction (m) from the macro-entry value
    // before the local blocker-passed / exit check may pass.
    double macro_local_leave_goal_progress_m = 1.5;
    // Forward-projection margin (m) behind the vehicle used by the
    // "blocker mainly behind" condition: a tracked boundary point counts
    // as behind when its projection onto the vehicle forward axis is
    // < -margin.
    double macro_local_blocker_behind_margin_m = 0.5;
    // Minimum fraction of the tracked blocker's observed boundary points
    // that must lie behind the vehicle (see above) for the "mainly
    // behind" condition to hold.
    double macro_local_blocker_behind_fraction = 0.6;
    // Local blocker track timeout (s): after this long without a CURRENT
    // patch association, predicted association and history back-fill are
    // disabled; only geometry-near re-association is accepted.  The exit
    // conditions still gate the real exit.
    double macro_local_track_timeout_s = 3.0;
    // Local guide candidate sampling range (m): the deterministic local
    // candidate set samples lookahead distances in this interval.
    double macro_local_frontier_min_distance_m = 1.5;
    double macro_local_frontier_max_distance_m = 4.0;
    // Local guide candidate sampling resolution.
    double macro_local_candidate_bearing_step_deg = 5.0;
    double macro_local_candidate_distance_step_m = 0.5;
    // Radius (m) of the bounded nearest-OCCUPIED search used for the
    // observed-clearance measurements (chord certification + the
    // guide_min_observed_clearance diagnostic).
    double macro_local_clearance_search_radius_m = 3.0;
    // Max number of observed boundary points kept in one local blocker
    // track (memory bound; the track is observation-only).
    int    macro_local_track_max_points = 2000;
    // Association radius (m): a newly observed cluster whose cells lie
    // within this distance of the tracked boundary set / the predicted
    // track position is treated as the SAME local blocker (never a truth
    // id — pure observation association).
    double macro_local_track_assoc_radius_m = 2.0;
    // Recovery prefix (m) of a guide chord that may lie inside the
    // handoff-inflated observed clearance shell.  The vehicle itself is
    // normally inside that shell when the macro triggers (it stopped at
    // the 30 Hz hard clearance), so the straight chord to the guide may
    // climb out of the shell for this short prefix; everything beyond it
    // (including the endpoint) must be handoff-clear.  0 disables the
    // recovery prefix entirely.
    double macro_local_recovery_prefix_m = 0.8;

    // ── v9: 5 Hz VISIBILITY TARGET CORRECTOR (local observability judge
    //        + target corrector) and target-encoding protocol ─────────
    // The 5 Hz expert no longer plans detour routes.  It only answers
    // "does the current FOV already contain enough information for the
    // 30 Hz expert to finish its own local avoidance?" and, when not,
    // temporarily corrects the tracked target (NORMAL_CORRECTION) or
    // forces a pure view rotation (TURN_LEFT / TURN_RIGHT).  The
    // EffectiveTargetAdapter converts the zero-order-held directive into
    // the LocalTarget world point for the C++ 30 Hz expert AND into the
    // body-frame unit direction + normalized distance that a future 30 Hz
    // student would consume.  All values are pure geometry / timing with
    // finiteness + range checks in params_io.
    // Number of ordinary in-FOV direction bins (NOT counting the special
    // TURN_LEFT / TURN_RIGHT classes).  Must be ODD (>= 3) so the bins
    // are symmetric around the 0° (forward) direction and include it
    // exactly.  Total student classes = direction_bin_count + 2
    // (class 0 = TURN_LEFT, classes 1..N = ordinary bins, class N+1 =
    // TURN_RIGHT).
    int    te_direction_bin_count = 11;
    // reserve_m of the normal-distance encoding:
    //   normal_distance = min(real_target_distance, R - reserve_m) / R
    // with R = observation/range_m.  The maximum normalized distance of an
    // ORDINARY target is therefore strictly < 1
    // (normal_distance_max = (R - reserve_m)/R ≈ 0.9167 for R=6, r=0.5),
    // which keeps ordinary classes distinguishable from the TURN classes
    // that carry an EXACT normalized distance of 1.0.
    double te_normal_distance_reserve_m = 0.5;
    // Initial margin (deg) of each bounded TURN step outside the FOV:
    //   left_bearing_body  = +(FOV/2 + turn_ray_margin_deg)
    //   right_bearing_body = -(FOV/2 + turn_ray_margin_deg)
    // The direction is then world-latched until it enters the FOV. The
    // same margin is used as the ordinary-bin coverage margin (the
    // ordinary bins symmetrically cover [-FOV/2 + margin, +FOV/2 - margin]).
    double te_turn_ray_margin_deg = 10.0;
    // Consecutive real 5 Hz cycles for which the enter condition must hold
    // before the corrector enters a correction episode.
    int    macro_correction_enter_stable_ticks = 1;
    // Minimum distance (m) from the vehicle of a certified local bypass /
    // observation frontier candidate used by the observability judgement
    // and by NORMAL_CORRECTION target selection.
    double macro_observable_frontier_min_distance_m = 1.5;
    // Minimum forward progress (m) toward the original goal that a local
    // bypass exit / observation frontier must provide to count as
    // "observable" (the 30 Hz expert would be able to make real progress).
    double macro_observable_frontier_min_progress_m = 0.5;
    // Minimum number of known-FREE cells that must extend BEYOND a
    // certified frontier point (along the same ray) before the exit counts
    // as non-truncated (not merely stopping at an UNKNOWN / FOV boundary).
    int    macro_observable_unknown_margin_cells = 3;

    // vehicle/referee thresholds
    double vehicle_goal_stop_speed_mps = 0.2;
    double vehicle_stationary_speed_mps = 0.05;

    // gui
    double gui_refresh_rate_hz = 30.0;
    double gui_default_speed = 1.0;
    std::vector<double> gui_speeds{0.1, 0.25, 0.5, 1.0, 2.0};
    bool gui_show_esdf = true;
    bool gui_show_truth_paths = true;
    bool gui_show_local_observation = true;
    bool gui_show_rejected_candidates = true;

    // logging (flight-log export directory + startup cleanup)
    std::string logging_output_directory = "~/.ros/il_2d_multiscale_debug_logs";
    bool logging_clear_on_start = true;
    std::string logging_filename_prefix = "flight_log_";

    uint64_t default_seed = 42;
};

// ═══════════════════════════════════════════════════════════════════
//  2D geometric primitives
// ═══════════════════════════════════════════════════════════════════
struct Obstacle2D {
    Vec2d center{0.0, 0.0};
    double radius = 0.0;
    int id = -1;
};

struct Scene2D {
    Vec2d min_bounds{-20.0, -20.0};
    Vec2d max_bounds{20.0, 20.0};
    std::vector<Obstacle2D> obstacles;
    uint64_t seed = 0;
    uint64_t scene_id = 0;
    bool valid = false;
};

struct VehicleState2D {
    Vec2d position{0.0, 0.0};
    double yaw = 0.0;
    Vec2d velocity_world{0.0, 0.0};  // world-frame velocity
    double yaw_rate = 0.0;
};

// ═══════════════════════════════════════════════════════════════════
//  Result of a bounded nearest-OCCUPIED-cell query on a LocalObservation.
//  `found=false` / distance=inf means no OCCUPIED cell inside the search
//  radius (the caller may treat that as "no observed obstacle nearby").
// ═══════════════════════════════════════════════════════════════════
struct NearestOccupiedResult {
    bool found = false;
    double distance = std::numeric_limits<double>::infinity();
    Vec2d cell_center{0.0, 0.0};
};

// ═══════════════════════════════════════════════════════════════════
//  Local observation (world-aligned grid with short-term history)
// ═══════════════════════════════════════════════════════════════════
struct LocalObservation {
    double resolution = 0.1;
    int width = 0, height = 0;
    Vec2d origin{0.0, 0.0};  // world position of cell (0,0)
    std::vector<CellState> cells;
    std::vector<uint32_t> age_ticks;  // ticks since last observed
    uint32_t max_age_ticks = 120;
    uint64_t tick = 0;

    bool valid() const { return width > 0 && height > 0 && !cells.empty(); }
    bool inGrid(int ix, int iy) const {
        return ix >= 0 && iy >= 0 && ix < width && iy < height;
    }
    size_t idx(int ix, int iy) const { return static_cast<size_t>(iy) * width + ix; }
    CellState at(int ix, int iy) const {
        return inGrid(ix, iy) ? cells[idx(ix, iy)] : CellState::UNKNOWN;
    }
    CellState atWorld(double x, double y) const {
        // Package-wide floor convention (see worldToGrid): same rule as
        // the FOV patch and the history map, so a query at a world point
        // always returns the same global cell the observation wrote.
        const GridIndex2D g = worldToGrid(Vec2d(x, y), origin, resolution);
        return at(g.ix, g.iy);
    }
    bool isKnownFree(double x, double y) const { return atWorld(x, y) == CellState::FREE; }
    bool isOccupied(double x, double y) const { return atWorld(x, y) == CellState::OCCUPIED; }

    /// Visit every observed OCCUPIED cell whose centre lies inside the
    /// closed radius-r disk around p.  The bounding-box loop keeps the
    /// search finite; the explicit distance test excludes box corners that
    /// lie outside the requested circular radius.
    template <typename Visitor>
    void forEachOccupiedWithin(const Vec2d& p, double search_radius,
                               Visitor&& visitor) const {
        if (!valid() || !(search_radius > 0.0) ||
            !std::isfinite(search_radius)) {
            return;
        }

        const double r = search_radius;
        const double r2 = r * r;
        const int ix0 = static_cast<int>(std::floor(
            (p.x() - r - resolution - origin.x()) / resolution));
        const int ix1 = static_cast<int>(std::ceil(
            (p.x() + r + resolution - origin.x()) / resolution));
        const int iy0 = static_cast<int>(std::floor(
            (p.y() - r - resolution - origin.y()) / resolution));
        const int iy1 = static_cast<int>(std::ceil(
            (p.y() + r + resolution - origin.y()) / resolution));

        for (int iy = std::max(0, iy0); iy <= std::min(height - 1, iy1);
             ++iy) {
            for (int ix = std::max(0, ix0); ix <= std::min(width - 1, ix1);
                 ++ix) {
                if (cells[idx(ix, iy)] != CellState::OCCUPIED) continue;
                const Vec2d centre(origin.x() + (ix + 0.5) * resolution,
                                   origin.y() + (iy + 0.5) * resolution);
                const double d2 = (centre - p).squaredNorm();
                if (d2 > r2 + 1e-12) continue;
                visitor(centre, std::sqrt(std::max(0.0, d2)));
            }
        }
    }

    /// Nearest observed OCCUPIED cell (centre + distance) inside the
    /// radius-r search neighbourhood, plus its world centre.  The search
    /// is BOUNDED: only cells whose centre lies within `search_radius` of
    /// `p` are scanned (search box enlarged by one cell so no boundary
    /// cell is missed).  UNKNOWN cells are never treated as occupied here;
    /// callers enforce the known-free constraint separately.  Returns
    /// found=false when no OCCUPIED cell is found inside the radius.
    NearestOccupiedResult nearestOccupied(const Vec2d& p,
                                          double search_radius) const {
        NearestOccupiedResult out;
        double best = std::numeric_limits<double>::infinity();
        Vec2d best_centre(0.0, 0.0);
        forEachOccupiedWithin(
            p, search_radius, [&](const Vec2d& centre, double distance) {
                if (distance < best) {
                    best = distance;
                    best_centre = centre;
                }
            }
        );
        if (best < std::numeric_limits<double>::infinity()) {
            out.found = true;
            out.distance = best;
            out.cell_center = best_centre;
        }
        return out;
    }

    /// Minimum distance from p to the centre of an observed OCCUPIED cell
    /// inside the radius-r search neighbourhood (reuses nearestOccupied).
    /// UNKNOWN cells are not treated as occupied here; callers enforce the
    /// known-free constraint separately.  Returns infinity when no
    /// occupied cell is found.
    double minClearanceToOccupied(const Vec2d& p, double r) const {
        return nearestOccupied(p, r).distance;
    }
};

// ═══════════════════════════════════════════════════════════════════
//  Local target (the ONLY goal information the 30 Hz planner receives)
// ═══════════════════════════════════════════════════════════════════
struct LocalTarget {
    Vec2d position{0.0, 0.0};
    bool valid = false;
    bool is_macro_guide = false;  // legacy compatibility field; v9 keeps it
                                  // false so 30 Hz receives no mode signal
    // Semantic directive update event (5 Hz ZOH boundary), used for
    // logging / visualization only. It does not follow the live clipped
    // PASS_THROUGH point or live pose re-expression of a TURN anchor. A change of this
    // counter alone NEVER resets planner memory.
    // v9: it equals the 5 Hz directive update_event — it changes ONLY on a
    // real 5 Hz directive change (type / side / NORMAL target / bounded
    // TURN anchor), never on a per-30Hz live coordinate transform.
    uint64_t update_event = 0;
    // MISSION revision.  Changed by the FSM ONLY on: new task / scene
    // reset and every formally accepted final navigation-goal revision.
    // v9: entering / refreshing / leaving a 5 Hz correction NEVER changes
    // it (the 30 Hz planner must not clear its internal memory because the
    // 5 Hz corrector intervened).  The 30 Hz planner resets its per-mission
    // memory (command continuity, stall window) only when this changes —
    // never on a plain update_event change.
    uint64_t mission_revision = 0;
    // The second channel of the public 30 Hz target contract. Ordinary
    // targets are strictly below 1; an exact value of 1 is the reserved
    // pure-rotation command. This is not a hidden correction-mode flag.
    double normalized_distance = 0.0;
};

// ═══════════════════════════════════════════════════════════════════
//  Trajectory / planner result
// ═══════════════════════════════════════════════════════════════════
struct Trajectory2D {
    std::vector<Vec2d> points;
    std::vector<double> yaw;
    std::vector<double> t;  // seconds from plan start
    bool valid = false;
};

struct PlannerResult {
    bool success = false;
    bool turn_mode = false;         // planner wants TURN_TO_TARGET
    FailureReason failure_reason = FailureReason::NONE;
    // ── Intent vs output (v5) ──────────────────────────────────────
    // intent_* = the LONG-TERM rollout intent (desired control, raw YAML
    //            samples or feedback laws).  vx_body/vy_body/yaw_rate are
    //            the EXECUTABLE OUTPUT: the one-control-period reachable
    //            command from the current state toward the intent — the
    //            only thing sent to the simulator / expert labels.
    double vx_body = 0.0;           // executable output: body-forward velocity
    double vy_body = 0.0;           // executable output: body-lateral velocity
    double yaw_rate = 0.0;          // executable output: commanded yaw rate
    double intent_vx_body = 0.0;    // rollout intent: body-forward velocity
    double intent_vy_body = 0.0;    // rollout intent: body-lateral velocity
    double intent_yaw_rate = 0.0;   // rollout intent: commanded yaw rate
    // ── Candidate-category / progress diagnostics (v5) ─────────────
    PlannerStatus planner_status = PlannerStatus::NO_SAFE_CANDIDATE;
    // Progress is reported at two levels.  candidate_progress_qualified is
    // the property of the best safe trajectory before any supervisory
    // override; output_progress_qualified describes the command that is
    // actually sent to the simulator.  progress_qualified is retained as
    // the backwards-compatible name for the FINAL output property and must
    // always equal output_progress_qualified.
    bool candidate_progress_qualified = false;
    bool output_progress_qualified = false;
    bool progress_qualified = false;
    // Progress over the target-limited nominal rollout (intent-driven,
    // ending at target capture / closest approach and never beyond it).
    double nominal_progress_m = 0.0;
    // Progress over the CERTIFIED executable safe prefix (what will really
    // be executed before the next 30 Hz replan).
    double executable_progress_m = 0.0;
    // Duration (s) of the certified executable safe prefix.
    double safe_prefix_duration_s = 0.0;
    // Magnitude of the executable output command.
    double selected_output_speed_mps = 0.0;
    // True when the selected executable output is (near-)zero and WHY it
    // was allowed (terminal_settling / turn_mode / emergency_brake /
    // stalled_without_progress / no_safe_motion_candidate).
    bool stationary_candidate_selected = false;
    std::string stationary_selection_reason;
    // True on the tick where the LocalTarget jumped beyond
    // target_discontinuity_reset_m and planner memory was reset.
    bool target_discontinuity_reset = false;
    // Mission revision of the LocalTarget used for this plan.
    uint64_t target_mission_revision = 0;
    Trajectory2D selected;
    std::vector<Trajectory2D> rejected_candidates;
    bool blocked_observed = false;         // observed obstacle blocking corridor
    bool immediate_avoidance = false;      // previously executed traj now blocked
    bool emergency_brake = false;          // stopping distance insufficient
    double min_observed_clearance = std::numeric_limits<double>::infinity();

    // ── Soft-clearance / dynamic-envelope diagnostics (v4) ──────────
    // Explicit "no measurement" value when no candidate was selected:
    // quiet_NaN (never 0, so a real 0 reading can never be mistaken for a
    // missing one).
    double selected_soft_min_clearance_m = std::numeric_limits<double>::quiet_NaN();
    // Max speed-dependent required clearance along the selected candidate
    // (handoff_clearance + closing*reaction + closing^2/(2*max_accel)).
    double selected_dynamic_required_clearance_m = std::numeric_limits<double>::quiet_NaN();
    // Max closing speed toward an observed obstacle along the candidate.
    double selected_closing_speed_mps = std::numeric_limits<double>::quiet_NaN();
    // Static macro-handoff base clearance used by this planner.
    double handoff_clearance_m = std::numeric_limits<double>::quiet_NaN();
    // True only when the PLAN OUTCOME is blocked by the dynamic envelope
    // (no executable progressing candidate), or the previously executed
    // trajectory suffix was invalidated by that envelope.  A rejection of
    // just one sampled candidate is diagnostic only and does not set this.
    bool dynamic_clearance_blocked = false;
    // True when the deterministic stall / control-oscillation detector
    // fired.  In direct mode this commands a safe brake and escalates to
    // the 5 Hz expert; under an active macro guide it is diagnostic only
    // when a safe progressing recovery candidate exists.
    bool local_limit_cycle_detected = false;
    // Number of DISTINCT candidate commands after the 30 Hz dynamic-window
    // projection + deterministic de-duplication.
    uint32_t dynamic_window_candidate_count = 0;

    // ── Per-tick candidate rejection diagnostics ────────────────────
    // Each REJECTED candidate counts exactly its FIRST decisive reason, so
    // the six counters sum to rejected_candidates.size().  Reset every
    // tick by construction (fresh PlannerResult); preview plans never
    // mutate these (they are local to the returned result).
    uint32_t reject_not_known_free = 0;
    uint32_t reject_outside_current_fov = 0;
    uint32_t reject_observed_clearance_too_small = 0;
    uint32_t reject_no_progress = 0;
    uint32_t reject_other = 0;
    // Rejected by the speed-dependent dynamic braking / handoff envelope
    // (INSUFFICIENT_BRAKING_CLEARANCE).  Counts as real observed blockage.
    uint32_t reject_insufficient_braking_clearance = 0;

    /// Total rejected candidates (== rejected_candidates.size()).
    uint32_t rejectedCount() const {
        return static_cast<uint32_t>(rejected_candidates.size());
    }
    /// Sum of the per-reason counters — must equal rejectedCount().
    uint32_t rejectCountSum() const {
        return reject_not_known_free + reject_outside_current_fov +
               reject_observed_clearance_too_small + reject_no_progress +
               reject_other + reject_insufficient_braking_clearance;
    }

    // ── Selected-candidate target-tracking diagnostics ─────────────
    // Filled for the BEST feasible candidate every tick (0.0 when no
    // candidate was selected / turn mode).  Preview plans never mutate
    // these (they live in the returned PlannerResult only).
    double target_bearing_error_deg = 0.0;            // |bearing| to LocalTarget
    double selected_terminal_heading_error_deg = 0.0; // heading at prefix end vs target dir
    double selected_velocity_alignment_error_deg = 0.0;  // velocity dir vs target dir
    double selected_cross_track_error_m = 0.0;        // mean lateral offset to local ref line
    double selected_cost_total = 0.0;
    double selected_cost_progress = 0.0;
    double selected_cost_clearance = 0.0;
    double selected_cost_smoothness = 0.0;
    double selected_cost_speed_change = 0.0;
    double selected_cost_yaw_rate_change = 0.0;
    double selected_cost_terminal_heading = 0.0;
    double selected_cost_velocity_alignment = 0.0;
    double selected_cost_cross_track = 0.0;
    double selected_cost_obstacle_risk = 0.0;

    // ── Continuous early-avoidance risk diagnostics (v7) ───────────
    // True when an observed obstacle intersects the local vehicle →
    // LocalTarget corridor ahead of the vehicle (independent of the
    // selected candidate).
    bool local_corridor_blocked = false;
    // Longitudinal distance (m) along the corridor to the first obstacle
    // that blocks it (NaN when the corridor is clear).
    double first_blocking_obstacle_distance =
        std::numeric_limits<double>::quiet_NaN();
    // Selected candidate's predicted closest clearance (m) to any observed
    // obstacle (closest approach of the nominal rollout); +inf when no
    // obstacle was in the risk field.
    double predicted_closest_clearance =
        std::numeric_limits<double>::quiet_NaN();
    // First predicted entry into the nominal-clearance shell (s, +inf when
    // the candidate never enters it).
    double time_to_collision = std::numeric_limits<double>::quiet_NaN();
    // Selected candidate's normalized obstacle-risk cost term [0,~1].
    double obstacle_risk_cost = 0.0;
    // Normalized avoidance strength of the selected candidate (== risk).
    double avoidance_strength = 0.0;
    // True when avoidance_strength >= lp_avoidance_active_threshold.
    bool avoidance_active = false;
    // Distance (m) from the vehicle to the current LocalTarget.
    double local_target_distance = 0.0;
};

// ═══════════════════════════════════════════════════════════════════
//  Audit flags — displayed in the GUI for information-leak inspection
// ═══════════════════════════════════════════════════════════════════
struct AuditFlags {
    // The first three are CONSTANT false by construction for the 30 Hz planner.
    bool used_truth_by_local_planner = false;
    bool used_global_esdf_by_local_planner = false;
    bool used_global_path_by_local_planner = false;
    // v8: always false — the 5 Hz expert no longer touches the global
    // truth ESDF at runtime (it is a pure local observer).
    bool macro_used_privileged_esdf = false;
    // v8: true whenever the 5 Hz macro expert ran using ONLY local
    // observations (current_patch + local_history_map + its own memory).
    bool macro_used_local_history_only = false;
    bool side_selected_from_visible_evidence = false;
    bool side_ambiguous_defaulted_right = false;
    // True whenever the side evidence was computed from the INSTANTANEOUS
    // FOV patch (current_patch), never from the merged history map.
    bool side_selected_using_current_patch = false;
    uint64_t local_target_update_event = 0;
    uint64_t macro_enter_event = 0;
    uint64_t macro_exit_event = 0;
    uint64_t obstacle_first_observed_event = 0;
    uint64_t immediate_avoidance_event = 0;
    uint64_t emergency_brake_event = 0;
};

// ═══════════════════════════════════════════════════════════════════
//  Route / task / evidence / transition
// ═══════════════════════════════════════════════════════════════════
struct Route2D {
    std::vector<Vec2d> waypoints;  // dense, evenly spaced
    double length = 0.0;
    bool valid = false;
    // ── Macro A* start-clearance recovery (defensive, §9) ───────────
    // When the macro route's A* start cell falls short of the route
    // clearance (0.1 m discretisation error) while the continuous start is
    // still above scene_safety_clearance, a short recovery connection
    // (start → nearest route-clear legal cell) is prepended.  That prefix
    // satisfies only the BASE safety clearance; the rest of the route
    // still satisfies scene_safety_clearance + route_clearance_margin.
    // Expressed with explicit C++ fields (never implicit via strings).
    double recovery_prefix_length = 0.0;   // arc length (m) of the recovery prefix
    bool start_clearance_recovery_used = false;
};

struct Task2D {
    Vec2d start{0.0, 0.0};
    Vec2d goal{0.0, 0.0};
    double initial_yaw = 0.0;
    uint64_t task_id = 0;
    uint64_t scene_id = 0;
    uint64_t seed = 0;
    bool valid = false;
};

struct SideEvidence {
    SideSelection selection = SideSelection::NONE;
    bool from_visible_evidence = false;
    bool ambiguous_defaulted_right = false;
    // AVERAGE visible free range actually used for the comparison
    // (m) — never an unnormalised total.
    double left_score = 0.0;
    double right_score = 0.0;
    std::string reason = "NONE";
};

// ═══════════════════════════════════════════════════════════════════
//  v9: 5 Hz target-correction types (local observability judge)
// ═══════════════════════════════════════════════════════════════════
//  The 5 Hz expert is a PURE LOCAL "visibility judge + target corrector".
//  It never reads the 30 Hz planner outcome (PlannerResult / PreviewResult
//  / FailureReason / PlannerStatus / previewPlan / turn_mode /
//  emergency_brake / blocked_observed / consecutive failures), the global
//  ESDF, the scene truth, global A* or the global left/right routes.  Its
//  only output is a zero-order-held TargetCorrectionDirective:
//    PASS_THROUGH        → the 30 Hz expert keeps tracking the ORIGINAL
//                          goal (its FOV already contains everything it
//                          needs to finish local avoidance itself);
//    NORMAL_CORRECTION   → temporarily retarget the 30 Hz expert at a
//                          quantized in-FOV frontier on the locked side;
//    TURN_LEFT / TURN_RIGHT → force a pure view rotation toward the
//                          FOV-external ray on the locked side until the
//                          local bypass becomes observable.
enum class TargetCorrectionType : uint8_t {
    PASS_THROUGH = 0,
    NORMAL_CORRECTION = 1,
    TURN_LEFT = 2,
    TURN_RIGHT = 3,
};

inline const char* targetCorrectionTypeName(TargetCorrectionType t) {
    switch (t) {
        case TargetCorrectionType::PASS_THROUGH: return "PASS_THROUGH";
        case TargetCorrectionType::NORMAL_CORRECTION: return "NORMAL_CORRECTION";
        case TargetCorrectionType::TURN_LEFT: return "TURN_LEFT";
        case TargetCorrectionType::TURN_RIGHT: return "TURN_RIGHT";
    }
    return "UNKNOWN";
}

/// The 5 Hz corrector's output, zero-order held between 5 Hz boundaries.
/// The EffectiveTargetAdapter converts it every 30 Hz tick into the
/// EncodedTargetInput (body direction unit vector + normalized distance
/// for the future 30 Hz student AND the LocalTarget world point for the
/// current C++ 30 Hz expert).
struct TargetCorrectionDirective {
    TargetCorrectionType type = TargetCorrectionType::PASS_THROUGH;
    bool valid = true;
    // Student direction class:
    //   0                 = TURN_LEFT (special class),
    //   1 .. N            = ordinary in-FOV bins ordered LEFT to RIGHT
    //                       (N = direction_bin_count),
    //   N + 1             = TURN_RIGHT (special class).
    // -1 for PASS_THROUGH (no direction label).
    int direction_token = -1;
    // Direction decoded at the 5 Hz decision instant (bin centre for an
    // ordinary target, initial FOV-external ray for a bounded TURN step).
    // During TURN the adapter re-expresses the latched world direction at
    // every live pose, so the actual 30 Hz body direction then converges.
    Vec2d decoded_direction_body{1.0, 0.0};
    // DECODED normalized distance (student label).  For ordinary classes
    // it is clamped to normal_distance_max < 1; for TURN classes it is
    // EXACTLY 1.0; for PASS_THROUGH it is meaningless (the adapter
    // recomputes the live value from the original goal).
    double normalized_distance = 0.0;
    // NORMAL_CORRECTION: world point locked during the 5 Hz period and
    // rebuilt from the QUANTIZED direction + clamped distance. Invalid for
    // PASS_THROUGH / TURN.
    Vec2d corrected_target_world{0.0, 0.0};
    bool corrected_target_world_valid = false;
    // TURN_LEFT / TURN_RIGHT: world-frame UNIT direction captured when the
    // bounded turn step is issued. Position drift cannot rotate this
    // direction; its body bearing changes only with live yaw. Invalid for
    // PASS_THROUGH / NORMAL_CORRECTION.
    Vec2d turn_direction_world{1.0, 0.0};
    bool turn_direction_world_valid = false;
    // Side locked for the current correction episode (NONE when not
    // correcting).  Never switches inside one episode.
    SideSelection locked_side = SideSelection::NONE;
    // Directive update event: increments ONLY when the directive value
    // changes on a real 5 Hz boundary (type / side / NORMAL target value).
    // Live pose re-expression of a latched world direction never bumps it.
    uint64_t update_event = 0;
    std::string reason = "PASS_THROUGH";
};

/// Local observability judgement of the 5 Hz corrector (deterministic,
/// local, causal — current FOV patch plus the decaying local history map).
struct AvoidanceObservability {
    // Original-goal direction inside the current FOV (raw, no margin).
    bool goal_inside_fov = false;
    // An OCCUPIED cell blocks the vehicle→original-goal local corridor
    // (checked only to perception range in the causal local map; UNKNOWN
    // is never treated as FREE).
    bool direct_corridor_blocked = false;
    // direct_corridor_blocked with an actual observed OCCUPIED cluster
    // (i.e. a real blocker, not merely occlusion).
    bool blocker_observed = false;
    // A clear, connected, known-free local bypass exit on each side is
    // fully visible inside the current FOV (enough clearance, positive
    // progress, not truncated by UNKNOWN / FOV boundary).
    bool left_bypass_observable = false;
    bool right_bypass_observable = false;
    // true ⇔ the current FOV already gives the 30 Hz expert everything it
    // needs for its own local avoidance:
    //   (goal_inside_fov && direct corridor clear) OR
    //   (left || right) certified bypass.
    bool local_avoidance_observable = false;
    // The needed exit region is cut off by the FOV boundary.
    bool fov_boundary_truncated = false;
    // The view toward the corridor / exit is occluded by UNKNOWN cells.
    bool unknown_occluded = false;
    // Side evidence scores (m): farthest certified frontier distance on
    // each side (0 when none) — observation-only.
    double left_score = 0.0;
    double right_score = 0.0;
    std::string reason = "NONE";
};

/// The adapter's per-30Hz-tick output — the exact information bottleneck
/// shared by the C++ 30 Hz expert (world target) and a future 30 Hz
/// student (body direction + normalized distance).
struct EncodedTargetInput {
    bool valid = false;
    // Body-frame unit direction (+X forward, +Y left).
    Vec2d direction_body{1.0, 0.0};
    // Normalized distance (0 .. 1).  Ordinary targets: clamped to
    // normal_distance_max < 1; TURN classes: EXACTLY 1.0; at the goal: 0.
    double normalized_distance = 0.0;
    // World point handed to the C++ 30 Hz expert as its LocalTarget.
    Vec2d effective_target_world{0.0, 0.0};
    bool effective_target_world_valid = false;
    TargetCorrectionType source_type = TargetCorrectionType::PASS_THROUGH;
};

/// FIXED physical homotopy reference captured at macro entry.
/// LEFT/RIGHT are interpreted ONLY relative to this axis and this
/// blocker centre while the blocker is being passed: vehicle movement
/// and navigation-goal changes NEVER redefine left/right.
struct HomotopyReference {
    bool valid = false;
    Vec2d entry_axis{0.0, 0.0};   // unit axis: entry pos → entry final goal
    Vec2d entry_blocker_center{0.0, 0.0};
    SideSelection side = SideSelection::NONE;
    double side_normal_sign = 0.0;  // +1 LEFT, -1 RIGHT
};

/// Outcome of associating the LOCAL blocker evidence with truth cylinders.
enum class BlockerAssociation : uint8_t {
    NONE = 0,
    MATCHED = 1,
    NO_MATCH = 2,
    AMBIGUOUS_MATCH = 3,
};

inline const char* blockerAssociationName(BlockerAssociation a) {
    switch (a) {
        case BlockerAssociation::NONE: return "NONE";
        case BlockerAssociation::MATCHED: return "MATCHED";
        case BlockerAssociation::NO_MATCH: return "NO_MATCH";
        case BlockerAssociation::AMBIGUOUS_MATCH: return "AMBIGUOUS_MATCH";
    }
    return "UNKNOWN";
}

/// LOCAL blocker EVIDENCE — derived ONLY from the current LocalObservation
/// (the visible OCCUPIED cluster).  It carries NO truth fields and is the
/// only thing the 5 Hz trigger/side-evidence path may use.  The PRIVILEGED
/// truth geometry lives in BlockerInfo and is kept strictly separate so
/// truth can never leak into the 30 Hz planner.
struct LocalBlockerEvidence {
    bool found = false;
    int cluster_id = -1;
    Vec2d visible_centroid{0.0, 0.0};  // centroid of the visible OCCUPIED cells
    double visible_radius = 0.0;       // bounding radius of those cells
    int visible_cell_count = 0;
    // World coordinates of the visible OCCUPIED cells.  NO truth ids are
    // ever stored here (see resolvePrivilegedBlocker).
    std::vector<Vec2d> visible_cells;
};

/// LOCAL blocker TRACK (v8) — the 5 Hz expert's long-term memory of the
/// CURRENT blocking obstacle.  It is built ONLY from real observations
/// (the current FOV patch + the merged local history map) and NEVER
/// stores truth ids / truth circles.  Its geometry (centroid / bounding
/// radius / outward normal / tangent directions) is estimated purely from
/// the observed OCCUPIED boundary cells, so changing an unobserved part
/// of the scene can never change the track.
struct LocalBlockerTrack {
    bool valid = false;
    // Local track id: a monotonic counter owned by the 5 Hz expert.  It is
    // NEVER a truth obstacle id.
    int64_t track_id = -1;
    uint64_t created_tick = 0;
    // Last tick on which a NEW observation was associated with this track.
    uint64_t last_observed_tick = 0;
    // Macro-entry reference (fixed for the whole episode).
    Vec2d hit_position{0.0, 0.0};
    double hit_goal_distance = 0.0;
    Vec2d entry_goal_axis{1.0, 0.0};  // FIXED unit axis: hit_position → goal
    SideSelection locked_side = SideSelection::NONE;
    // Accumulated observed OCCUPIED boundary cells (world coords).
    std::vector<Vec2d> boundary_points;
    Vec2d centroid{0.0, 0.0};
    double bounding_radius = 0.0;
    Vec2d nearest_boundary_point{0.0, 0.0};
    double nearest_boundary_distance =
        std::numeric_limits<double>::infinity();
    // Local outward normal at the nearest boundary point (estimated from
    // the observed points — never from a truth circle).
    Vec2d outward_normal{0.0, 0.0};
    // Tangent directions of travel for the two绕行 sides (unit vectors,
    // estimated from the observed boundary, see MacroExpert5Hz):
    //   tangent_left  = rot2(outward_normal, -90°)  (LEFT绕 / CW orbit)
    //   tangent_right = rot2(outward_normal, +90°)  (RIGHT绕 / CCW orbit)
    Vec2d tangent_left{0.0, 0.0};
    Vec2d tangent_right{0.0, 0.0};
    // Cumulative detour progress diagnostics (m), relative to the FIXED
    // entry axis: monotonic forward projection and monotonic |lateral|.
    double detour_along_progress_m = 0.0;
    double detour_lateral_progress_m = 0.0;
    // Diagnostics filled by the FSM / exit path every 5 Hz tick.
    bool blocker_behind = false;
    bool goal_corridor_clear = false;
};

/// Result of the local-causal guide generation (v8).  Carries the local
/// known-free routes (published on the LEGACY left/right/locked topics —
/// the content is now local, never privileged) and the hard-certification
/// diagnostics of the delivered LocalTarget, plus body-frame supervision
/// quantities for training / logging.
struct LocalGuideResult {
    LocalTarget target;
    // Local candidate routes (compatibility topics only; NOT privileged).
    Route2D left_route;     // best certified candidate on the LEFT side
    Route2D right_route;    // best certified candidate on the RIGHT side
    Route2D locked_route;   // [vehicle → chosen guide] local known-free route
    // Guide arc / lookahead along the locked local route (m).
    double lookahead_used = 0.0;
    double route_progress = -1.0;
    // Why the guide was (re)selected this 5 Hz tick: local_tangent_advance
    // / local_frontier_advance / local_fov_edge_turn / local_hysteresis_hold
    // / local_turning_hold / local_backward_jump_hold /
    // local_shortened_lookahead / local_no_safe_guide_hold /
    // local_no_safe_guide_stop / local_no_blocker_evidence.  Empty when
    // the macro expert did not run.
    std::string update_reason = "";
    // Hard-certification diagnostics for the delivered guide.
    bool guide_inside_current_fov = false;
    bool guide_endpoint_known_free = false;
    bool guide_chord_known_free = false;
    double guide_min_observed_clearance =
        std::numeric_limits<double>::infinity();
    bool local_macro_route_valid = false;
    // Body-frame supervision (body +X forward, +Y left) of the guide.
    double relative_target_x_body = 0.0;
    double relative_target_y_body = 0.0;
    double target_bearing_rad = 0.0;
    double target_distance_m = 0.0;
};

/// PRIVILEGED blocker — built from local evidence ASSOCIATED to truth
/// cylinders via the Scene2D/global ESDF. center/radius are copied from
/// the one dominant matched truth cylinder (never a merged multi-cylinder
/// bounding circle). NEVER delivered to the 30 Hz planner.
struct BlockerInfo {
    bool found = false;
    int cluster_id = -1;  // display only (links back to the local cluster)
    Vec2d center{0.0, 0.0};   // truth-geometry bounding circle centre
    double radius = 0.0;      // truth-geometry bounding circle radius
    std::vector<int> obstacle_ids;  // associated truth obstacle ids
    BlockerAssociation association = BlockerAssociation::NONE;
};

/// Pure-local macro exit check (v8).  Every condition is computed from
/// local observations + the local blocker track + the 30 Hz preview —
/// never from the global ESDF, global route progress or truth geometry.
struct LocalExitCheck {
    // 1) The final goal re-entered the FOV with margin.
    bool goal_in_fov_margin = false;
    // 2) The 30 Hz preview (local_history_map) can independently produce a
    //    SAFE AND PROGRESSING trajectory to the goal, or a safe
    //    final-goal TERMINAL_SETTLING trajectory.
    bool local_precheck_ok = false;
    // 3) The current local blocker no longer intersects the vehicle→goal
    //    local corridor (pure observation).
    bool blocker_not_in_corridor = false;
    // 4) The blocker's observed boundary is mainly behind the vehicle.
    bool blocker_behind = false;
    // 5) Goal distance reduced by at least local_leave_goal_progress_m
    //    relative to the macro-entry value.
    bool goal_progress_made = false;
    // 6) Not emergency braking / not recovering turn.
    bool not_emergency_or_turn = false;
    // Diagnostic detail of the 30 Hz preview.
    bool local_has_progressing_trajectory = false;
    double local_executable_progress_m = 0.0;
    double local_safe_prefix_duration_s = 0.0;
    double local_output_speed_mps = 0.0;
    bool all() const {
        return goal_in_fov_margin && local_precheck_ok &&
               blocker_not_in_corridor && blocker_behind &&
               goal_progress_made && not_emergency_or_turn;
    }
};

struct TransitionRecord {
    bool happened = false;
    FsmState prev = FsmState::DIRECT_LOCAL;
    FsmState curr = FsmState::DIRECT_LOCAL;
    std::string reason = "";
    uint64_t tick = 0;
    uint32_t failure_count_30hz = 0;
    uint32_t macro_stable_exit_count = 0;
    SideSelection side = SideSelection::NONE;
    int blocker_id = -1;
    uint64_t local_target_update_event = 0;
};

}  // namespace il_2d_multiscale_debug
