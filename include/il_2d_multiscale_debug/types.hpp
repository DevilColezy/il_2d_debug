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
// ═══════════════════════════════════════════════════════════════════
enum class FsmState : uint8_t {
    DIRECT_LOCAL = 0,
    TURN_TO_TARGET = 1,
    LOCAL_BLOCKED_PENDING = 2,
    MACRO_SELECT_SIDE = 3,
    MACRO_GUIDANCE = 4,
    MACRO_EXIT_PENDING = 5,
    GOAL_REACHED = 6,
    TASK_INVALID = 7,
    COLLISION = 8,
    TIMEOUT = 9,
};

inline const char* fsmStateName(FsmState s) {
    switch (s) {
        case FsmState::DIRECT_LOCAL: return "DIRECT_LOCAL";
        case FsmState::TURN_TO_TARGET: return "TURN_TO_TARGET";
        case FsmState::LOCAL_BLOCKED_PENDING: return "LOCAL_BLOCKED_PENDING";
        case FsmState::MACRO_SELECT_SIDE: return "MACRO_SELECT_SIDE";
        case FsmState::MACRO_GUIDANCE: return "MACRO_GUIDANCE";
        case FsmState::MACRO_EXIT_PENDING: return "MACRO_EXIT_PENDING";
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
    SAFE_HOLD = 1,                    // safe but no progress (only selectable near goal)
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
    int    scene_min_obstacles = 0, scene_max_obstacles = 20;
    double scene_min_radius = 0.1, scene_max_radius = 4.0;
    std::string scene_radius_distribution = "log_uniform";
    double scene_safety_clearance = 0.5;
    double scene_passage_margin = 0.2;
    double scene_boundary_margin = 0.5;
    int    scene_max_attempts_per_obstacle = 60;
    int    scene_max_total_scene_attempts = 24;

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
    double cost_w_terminal_heading = 1.5;
    double cost_w_velocity_alignment = 1.2;
    double cost_w_cross_track = 1.0;
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
    int    macro_exit_stable_ticks = 3;
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
    bool is_macro_guide = false;  // true while a 5 Hz macro guide is active
    // Numeric VALUE update event (5 Hz ZOH boundary).  Increments whenever
    // the delivered target value changes; used for logging / visualization
    // only.  A change of this counter alone NEVER resets planner memory.
    uint64_t update_event = 0;
    // MISSION revision.  Changed by the FSM on: new task / reset, every
    // formally accepted final navigation-goal revision, and entering /
    // leaving macro mode.  The 30 Hz planner resets
    // its per-mission memory (command continuity, stall window) only when
    // this changes — never on a plain update_event change.
    uint64_t mission_revision = 0;
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
    // Progress over the FULL 2.5 s nominal rollout (intent-driven).
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
    bool macro_used_privileged_esdf = false;
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

struct MacroExitCheck {
    bool goal_in_fov_margin = false;
    // 30 Hz preview produced a SAFE AND PROGRESSING trajectory to the goal,
    // or a safe final-goal TERMINAL_SETTLING trajectory.  A generic safe
    // hold/stop away from the final-goal convergence region is not enough.
    bool local_precheck_ok = false;
    // Diagnostic detail of the 30 Hz preview (v5).
    bool local_has_progressing_trajectory = false;
    double local_executable_progress_m = 0.0;
    double local_safe_prefix_duration_s = 0.0;
    double local_output_speed_mps = 0.0;
    bool blocker_passed = false;
    bool not_emergency_or_turn = false;
    bool all() const {
        return goal_in_fov_margin && local_precheck_ok && blocker_passed &&
               not_emergency_or_turn;
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
