#pragma once
/// @file   hierarchical_expert_fsm.hpp
/// @brief  Two-level expert state machine (v9).
///
/// v9: the "macro detour planner" layers are GONE.  The 5 Hz
/// VisibilityTargetCorrector runs on every 5 Hz boundary (tick % 6 == 0)
/// inside the single 30 Hz step — independent of the 30 Hz outcome — and
/// produces a zero-order-held TargetCorrectionDirective.  The
/// EffectiveTargetAdapter converts that directive EVERY 30 Hz tick into
/// the LocalTarget the 30 Hz planner sees (world point for the C++ expert,
/// body direction + normalized distance for the future student).
///
/// States: DIRECT_LOCAL, TURN_TO_TARGET, GOAL_REACHED, TASK_INVALID,
/// COLLISION, TIMEOUT.  The 30 Hz planner keeps its own TURN_TO_TARGET
/// hysteresis; the 5 Hz layer never reads the 30 Hz result.

#include "il_2d_multiscale_debug/local_planner_30hz.hpp"
#include "il_2d_multiscale_debug/macro_expert_5hz.hpp"
#include "il_2d_multiscale_debug/effective_target_adapter.hpp"
#include "il_2d_multiscale_debug/types.hpp"

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace il_2d_multiscale_debug {

/// Everything the FSM needs from the outside world for one tick.
struct FsmInput {
    const Task2D& task;
    const VehicleState2D& state;
    /// INSTANTANEOUS FOV patch of THIS tick (FovRaycaster2D output,
    /// before merging).  Only the 5 Hz VisibilityTargetCorrector reads it.
    const LocalObservation& current_patch;
    /// Merged short-term HISTORY map (ObservedGrid2D).  Only the 30 Hz
    /// planner reads it; the 5 Hz corrector's v9 judgement uses the
    /// current FOV patch.
    const LocalObservation& history;
    uint64_t tick;
    bool collision;
};

/// Result of one tick, consumed by the simulator + node for publishing.
struct FsmStepOutput {
    FsmState state = FsmState::DIRECT_LOCAL;
    FsmState prev_state = FsmState::DIRECT_LOCAL;
    PlannerResult local;
    LocalTarget local_target;
    bool local_target_updated = false;  // directive update_event changed
                                        // on this tick (5 Hz boundary only)

    // ── v9: 5 Hz target correction + effective-target encoding ──────
    // The adapter output actually handed to the 30 Hz planner this tick.
    uint8_t target_correction_type = 0;  // TargetCorrectionType
    std::string target_correction_type_name = "PASS_THROUGH";
    bool target_correction_active = false;
    // Student direction class (0=TURN_LEFT, 1..N ordinary LEFT-to-RIGHT,
    // N+1=TURN_RIGHT; -1 for PASS_THROUGH).
    int32_t target_direction_token = -1;
    // Body-frame unit direction + normalized distance actually executed
    // (the 30 Hz student labels; every real 30 Hz tick).
    double target_direction_x_body = 1.0;
    double target_direction_y_body = 0.0;
    double target_distance_normalized = 0.0;
    // Effective world target handed to the C++ 30 Hz expert.
    double effective_target_x = 0.0;
    double effective_target_y = 0.0;
    bool effective_target_world_valid = false;
    // The original final goal the 5 Hz corrector judged against.
    Vec2d original_goal{0.0, 0.0};
    // ── 5 Hz local observability diagnostics ────────────────────────
    bool observability_goal_inside_fov = false;
    bool observability_direct_corridor_blocked = false;
    bool observability_blocker_observed = false;
    bool observability_left_bypass_visible = false;
    bool observability_right_bypass_visible = false;
    bool observability_local_avoidance_observable = false;
    bool observability_fov_boundary_truncated = false;
    bool observability_unknown_occluded = false;
    std::string observability_reason = "NONE";
    double observability_left_score = 0.0;
    double observability_right_score = 0.0;
    // ── Correction events ───────────────────────────────────────────
    uint64_t correction_enter_event = 0;
    uint64_t correction_exit_event = 0;
    uint64_t correction_update_event = 0;

    // ── LEGACY macro fields (v9: always invalid/empty; run-time DECISION
    //    use is REMOVED — kept only for message/CSV/GUI compatibility) ──
    bool macro_active = false;  // legacy: always false
    SideSelection side = SideSelection::NONE;  // v9: corrector locked side
    SideEvidence evidence;                     // legacy: empty
    Route2D left_route;                        // legacy: empty
    Route2D right_route;                       // legacy: empty
    Route2D locked_route;                      // legacy: empty
    BlockerInfo blocker;                       // legacy: empty
    LocalBlockerEvidence local_blocker;        // legacy: empty
    BlockerAssociation blocker_association = BlockerAssociation::NONE;
    bool blocker_passed_latched = false;       // legacy: always false
    bool start_clearance_recovery_used = false;  // legacy: always false
    double entry_vehicle_progress = -1.0;      // legacy
    double entry_blocker_progress = -1.0;      // legacy
    double entry_projection_dist = -1.0;       // legacy
    int32_t entry_segment_index = -1;          // legacy
    double entry_progress_delta = 0.0;         // legacy
    double entry_progress_max_delta = 0.0;     // legacy
    double macro_route_progress = -1.0;        // legacy
    double macro_guide_lookahead = 0.0;        // legacy
    std::string macro_guide_update_reason = "";  // legacy
    double macro_no_progress_duration = 0.0;   // legacy
    bool macro_used_local_history_only = true;  // legacy (5 Hz is local)
    bool macro_guide_inside_current_fov = false;  // legacy
    bool macro_guide_endpoint_known_free = false; // legacy
    bool macro_guide_chord_known_free = false;    // legacy
    double macro_guide_min_observed_clearance =
        std::numeric_limits<double>::infinity();   // legacy
    bool local_blocker_track_valid = false;        // legacy
    int32_t local_blocker_track_id = -1;           // legacy
    bool local_blocker_behind = false;             // legacy
    bool local_goal_corridor_clear = false;        // legacy
    double local_leave_progress_m = 0.0;           // legacy
    bool local_macro_route_valid = false;          // legacy
    // Body-frame supervision of the ACTUAL executed direction (v9: the
    // adapter's encoded body direction / distance).
    double relative_target_x_body = 1.0;
    double relative_target_y_body = 0.0;
    double target_bearing_deg = 0.0;
    double target_distance_m = 0.0;
    // LOCAL_UNKNOWN_RECOVERY diagnostic (UNKNOWN-driven; NEVER feeds any
    // 5 Hz decision).
    uint32_t unknown_recovery_ticks = 0;
    bool unknown_recovery_active = false;
    uint32_t unknown_recovery_episode_count = 0;
    TransitionRecord transition;
    AuditFlags audit;
    bool macro_tick_ran = false;  // 5 Hz boundary executed this tick
    uint64_t macro_tick_event = 0;  // cumulative 5 Hz boundary counter
    uint32_t consecutive_failures_30hz = 0;  // diagnostic only
    uint32_t macro_stable_exit_count = 0;    // legacy: always 0
};

class HierarchicalExpertFsm {
public:
    explicit HierarchicalExpertFsm(const Params2D& p);

    /// Reset all state (call on every new task / scene).
    void reset(const Task2D& task, uint64_t tick);

    /// Advance one 30 Hz tick.  The 5 Hz corrector runs at tick % 6 == 0;
    /// the EffectiveTargetAdapter runs every tick.
    FsmStepOutput step(const FsmInput& in);

    /// Force the COLLISION terminal state (called by the simulator after
    /// the swept check).  Updates `out` in place; does not re-run planning.
    void forceCollision(FsmStepOutput& out);

    /// Formally accept a NEW final navigation goal at a 5 Hz boundary.
    /// v9: increments mission_revision_ (a formally accepted new final
    /// goal is a new mission contract), resets the 5 Hz correction state
    /// and bumps the directive update event.  It never resets the 30 Hz
    /// planner beyond the mission_revision contract.
    void acceptNewGoal(const Vec2d& new_goal, uint64_t tick);

    FsmState state() const { return state_; }
    /// Directive update event (the only local-target event source).
    uint64_t effectiveLocalTargetEvent() const {
        return corrector_.directiveUpdateEvent();
    }
    /// Cumulative 5 Hz boundary counter (strictly increases on each
    /// boundary tick; used by stepToNext5Hz as the stop condition).
    uint64_t macroTickEvent() const { return macro_tick_event_; }
    /// Monotonic counter of formally-accepted final goals (visualization).
    uint64_t acceptedGoalEvent() const { return goal_revision_; }
    /// Remaining correction re-entry hysteresis, in 30 Hz ticks (0 = free).
    int reentryGuardTicks() const { return reentry_guard_; }
    /// The 5 Hz corrector (diagnostics / GUI).
    const VisibilityTargetCorrector& corrector() const { return corrector_; }

private:
    bool isTerminal(FsmState s) const;
    void transition(FsmStepOutput& out, FsmState next,
                    const std::string& reason);
    /// Fill the complete per-tick observability block: v9 correction +
    /// encoding fields, observability diagnostics, events, the legacy
    /// (empty) macro fields, counters, local target, audit and the last
    /// transition — so EVERY path (including terminal) stays observable.
    void fillObservability(FsmStepOutput& out) const;
    bool goalReached(const FsmInput& in) const;
    /// 30 Hz failure / UNKNOWN-recovery bookkeeping.  v9: DIAGNOSTIC ONLY
    /// — the 5 Hz corrector never reads these counters.
    void updateFailureBookkeeping(const FsmStepOutput& out,
                                  const FsmInput& in);
    /// Build the direction+distance-equivalent LocalTarget tagged with the
    /// current mission revision and directive event — the single place that
    /// constructs the 30 Hz planner contract.
    LocalTarget makeLocalTarget(const EncodedTargetInput& encoded) const;

    Params2D p_;
    FsmState state_ = FsmState::DIRECT_LOCAL;
    uint64_t tick_ = 0;

    LocalPlanner30Hz local_planner_;
    VisibilityTargetCorrector corrector_;
    EffectiveTargetAdapter adapter_;

    // ── v9 per-tick state ───────────────────────────────────────────
    // The zero-order-held 5 Hz directive and its per-tick encoding.
    TargetCorrectionDirective directive_;
    EncodedTargetInput last_encoded_;
    bool directive_updated_ = false;
    // The directive update event actually delivered to the 30 Hz planner
    // on the PREVIOUS tick (used to flag local_target_updated, including
    // goal-acceptance bumps that happen before step()).
    uint64_t last_delivered_event_ = 0;
    Vec2d last_original_goal_{0.0, 0.0};

    // ── Diagnostics (30 Hz failure / UNKNOWN recovery, goal revisions) ──
    uint32_t consecutive_failures_ = 0;
    uint64_t failure_start_tick_ = 0;
    uint32_t unknown_recovery_ticks_ = 0;
    uint32_t unknown_recovery_episode_count_ = 0;
    uint64_t goal_revision_ = 0;
    // MISSION revision: increments ONLY on a formally accepted new final
    // goal (and is reset to 0 by reset()).  5 Hz correction enter /
    // refresh / exit NEVER change it.
    uint64_t mission_revision_ = 0;
    // Display mirror of the corrector's re-entry guard (30 Hz ticks).
    int reentry_guard_ = 0;
    // Cumulative 5 Hz boundary counter.
    uint64_t macro_tick_event_ = 0;
    TransitionRecord last_transition_;
    AuditFlags audit_;
};

}  // namespace il_2d_multiscale_debug
