#pragma once
/// @file   hierarchical_expert_fsm.hpp
/// @brief  Explicit two-level expert state machine.
///
/// States: DIRECT_LOCAL, TURN_TO_TARGET, LOCAL_BLOCKED_PENDING,
/// MACRO_SELECT_SIDE, MACRO_GUIDANCE, MACRO_EXIT_PENDING, GOAL_REACHED,
/// TASK_INVALID, COLLISION, TIMEOUT.
///
/// The 5 Hz macro expert runs EXACTLY on tick % 6 == 0 inside the single
/// 30 Hz step — there are no two drifting wall timers.  The macro target
/// is zero-order held between 5 Hz boundaries.  Changing the goal never
/// resets the vehicle dynamics, the planner hysteresis or the local map
/// history (the 30 Hz planner only sees the new local target).

#include "il_2d_multiscale_debug/local_planner_30hz.hpp"
#include "il_2d_multiscale_debug/macro_expert_5hz.hpp"
#include "il_2d_multiscale_debug/types.hpp"

namespace il_2d_multiscale_debug {

/// Everything the FSM needs from the outside world for one tick.
struct FsmInput {
    const Scene2D& scene;
    const TruthEsdf2D& esdf;
    const Task2D& task;
    const VehicleState2D& state;
    /// INSTANTANEOUS FOV patch of THIS tick (FovRaycaster2D output,
    /// before merging).  Only the 5 Hz macro expert reads it
    /// (identifyBlocker / selectSideFromVisibleEvidence).
    const LocalObservation& current_patch;
    /// Merged short-term HISTORY map (ObservedGrid2D).  Only the 30 Hz
    /// planner (and the macro exit preview / braking checks that reason
    /// about the planner's own view) reads it.
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
    bool local_target_updated = false;
    bool macro_active = false;
    SideSelection side = SideSelection::NONE;
    SideEvidence evidence;
    Route2D left_route;
    Route2D right_route;
    Route2D locked_route;
    BlockerInfo blocker;  // privileged truth geometry (display/route)
    LocalBlockerEvidence local_blocker;  // observation-only evidence
    BlockerAssociation blocker_association = BlockerAssociation::NONE;
    bool blocker_passed_latched = false;
    // Macro A* start-clearance recovery was used on the current locked
    // route (the route begins with a short base-clearance recovery prefix).
    bool start_clearance_recovery_used = false;
    double entry_vehicle_progress = -1.0;      // arc length along fixed route
    double entry_blocker_progress = -1.0;      // fixed blocker arc length
    double entry_projection_dist = -1.0;       // lateral dist to fixed route
    int32_t entry_segment_index = -1;
    double entry_progress_delta = 0.0;
    double entry_progress_max_delta = 0.0;
    // ── Rolling macro guide diagnostics (v7) ───────────────────────
    // Guide arc (m) along the CURRENT locked route at which the guide sits.
    double macro_route_progress = -1.0;
    // Arc-length lookahead (m) actually used to select the guide.
    double macro_guide_lookahead = 0.0;
    // Why the guide was (re)selected this 5 Hz tick: normal_advance /
    // min_distance_advance / fov_clamp / chord_clip / hysteresis_hold /
    // fallback_goal* / no_progress_advance.
    std::string macro_guide_update_reason = "";
    // Continuous time (s) the vehicle has failed to translate while macro
    // guidance is active (diagnostic + re-advance trigger).
    double macro_no_progress_duration = 0.0;
    // LOCAL_UNKNOWN_RECOVERY diagnostic: consecutive NO_SAFE_CANDIDATE
    // (UNKNOWN-driven, NOT observed blockage) 30 Hz ticks.  NO_SAFE_CANDIDATE
    // never triggers the 5 Hz macro expert and never counts as blockage.
    uint32_t unknown_recovery_ticks = 0;
    bool unknown_recovery_active = false;
    uint32_t unknown_recovery_episode_count = 0;
    TransitionRecord transition;
    AuditFlags audit;
    bool macro_tick_ran = false;
    uint64_t macro_tick_event = 0;  // cumulative 5 Hz boundary counter
    uint32_t consecutive_failures_30hz = 0;
    uint32_t macro_stable_exit_count = 0;
};

class HierarchicalExpertFsm {
public:
    explicit HierarchicalExpertFsm(const Params2D& p);

    /// Reset all state (call on every new task / scene).
    void reset(const Task2D& task, uint64_t tick);

    /// Advance one 30 Hz tick.
    FsmStepOutput step(const FsmInput& in);

    /// Force the COLLISION terminal state (called by the simulator after
    /// the swept check).  Updates `out` in place; does not re-run planning.
    void forceCollision(FsmStepOutput& out);

    /// Formally accept a NEW final navigation goal at a 5 Hz boundary.
    /// Issues ONE effective local-target event when the visible target
    /// value actually changes; in macro mode keeps the locked side and the
    /// CURRENT blocker (never resets it) and rebuilds the locked-side
    /// route + macro target.  Marks the task TASK_INVALID (with a correct
    /// previous/current transition record) if the locked side becomes
    /// globally infeasible — never silently switches sides.
    void acceptNewGoal(const Scene2D& scene, const TruthEsdf2D& esdf,
                       const VehicleState2D& state, const Vec2d& new_goal,
                       uint64_t tick);

    FsmState state() const { return state_; }
    uint64_t effectiveLocalTargetEvent() const {
        return effective_local_target_event_;
    }
    /// Cumulative 5 Hz boundary counter (strictly increases on each
    /// boundary tick; used by stepToNext5Hz as the stop condition).
    uint64_t macroTickEvent() const { return macro_tick_event_; }
    /// Monotonic counter of formally-accepted final goals (visualization).
    uint64_t acceptedGoalEvent() const { return goal_revision_; }
    /// Remaining macro re-entry guard, measured in 30 Hz ticks (0 = free).
    int reentryGuardTicks() const { return reentry_guard_; }

private:
    bool isTerminal(FsmState s) const;
    void transition(FsmStepOutput& out, FsmState next, const std::string& reason);
    /// Fill the full macro observability block (side / evidence / routes /
    /// blockers / association / latched progress / counters / local
    /// target / macro_active / audit / last transition) so EVERY terminal
    /// path and the final step output stays complete.
    void fillMacroObservability(FsmStepOutput& out) const;
    bool canEnterMacro(const FsmStepOutput& out, const FsmInput& in) const;
    void runMacroDecision(FsmStepOutput& out, const FsmInput& in);
    void runMacroGuidance(FsmStepOutput& out, const FsmInput& in);
    bool goalReached(const FsmInput& in) const;

    /// Increment `effective_local_target_event_` ONLY when the delivered
    /// local-target value actually changed (beyond the configured
    /// tolerance).  Returns true when an event was issued.
    bool issueLocalTargetEvent(const Vec2d& value);

    /// Build a LocalTarget tagged with the CURRENT mission revision and
    /// effective local-target event — the single place that constructs the
    /// 30 Hz planner contract.
    LocalTarget makeLocalTarget(const Vec2d& pos, bool macro_guide) const;

    /// Continuous arc-length projection of `world` onto `route`: walks the
    /// polyline segment by segment (NOT nearest-waypoint), sets
    /// `progress` (m along the route, clamped to [0, total_length]) and
    /// `lateral_dist` (unsigned perpendicular distance).  Returns false
    /// when the route has no usable waypoints.
    bool projectOntoRoute(const Route2D& route, const Vec2d& world,
                          double& progress, double& lateral_dist) const;

    /// Update the monotonic vehicle progress along the FIXED entry
    /// reference route.  Search is restricted to a window around the last
    /// matched segment / the last progress (adjacent segments or a
    /// reasonable arc-length window) and capped by max_speed × 5 Hz
    /// interval (+ tolerance), so a full-route nearest-point + max() can
    /// never jump to a far-away similar segment.  When the lateral
    /// projection exceeds `macro_blocker_projection_max_dist_m` the old
    /// progress is kept.  Sets blocker_passed_latched_ once the progress
    /// passes the fixed blocker arc length + clear distance.
    void updateEntryProgress(const Vec2d& pos);

    Params2D p_;
    FsmState state_ = FsmState::DIRECT_LOCAL;
    uint64_t tick_ = 0;

    LocalPlanner30Hz local_planner_;
    MacroExpert5Hz macro_expert_;
    LeftRightRoutePlanner route_planner_;

    // macro state
    SideSelection side_ = SideSelection::NONE;
    SideEvidence evidence_;
    Route2D left_route_, right_route_, locked_route_;
    BlockerInfo blocker_;
    LocalBlockerEvidence local_evidence_;
    LocalTarget macro_target_;
    uint32_t consecutive_failures_ = 0;
    uint64_t failure_start_tick_ = 0;
    uint32_t macro_stable_exit_count_ = 0;
    // Re-entry hysteresis guard, unit = 30 Hz TICKS.  Decremented (with
    // saturation, never below 0) on EVERY non-terminal 30 Hz step, not
    // only on 5 Hz boundaries.  30 ticks ≈ 1 s.
    int reentry_guard_ = 0;
    // Unique, monotonic event number handed to the 30 Hz planner whenever
    // its local-target VALUE changes (see file comment).
    uint64_t effective_local_target_event_ = 0;
    // last local-target value actually delivered to the 30 Hz planner
    Vec2d last_delivered_target_{0.0, 0.0};
    bool last_delivered_valid_ = false;
    // monotonic final-goal revision counter (accepted goals)
    uint64_t goal_revision_ = 0;
    // ── Mission revision (v5) ──────────────────────────────────────
    // Incremented on every formally accepted final-goal revision, entering
    // macro guidance and exiting macro guidance.  reset()/new-task resets
    // both this counter and the local planner directly.  Handed to the 30 Hz planner via LocalTarget
    // mission_revision; the planner resets its per-mission memory (command
    // continuity + stall window) only when this changes — never on a plain
    // local-target update_event change (rolling 5 Hz target networks).
    uint64_t mission_revision_ = 0;
    // macro-entry reference for the blocker-passed check — FIXED at entry
    // (the locked route + the blocker progress along it), never recomputed
    // from the ever-changing goal direction.
    Route2D entry_reference_route_;
    bool entry_reference_valid_ = false;
    double entry_blocker_route_progress_ = -1.0;  // arc length to blocker ctr
    double entry_vehicle_progress_ = -1.0;        // monotonic vehicle progress
    double entry_vehicle_projection_dist_ = -1.0; // last lateral projection
    int entry_last_segment_index_ = -1;           // last matched route segment
    double entry_progress_delta_ = 0.0;
    double entry_progress_max_delta_ = 0.0;
    uint64_t entry_progress_update_tick_ =
        std::numeric_limits<uint64_t>::max();
    // Rolling macro-guide no-progress bookkeeping (v7): how long the vehicle
    // has failed to translate while macro guidance is active, and the last
    // sampled vehicle position for the per-5 Hz-interval displacement.
    double macro_no_progress_duration_ = 0.0;
    Vec2d last_macro_vehicle_pos_{0.0, 0.0};
    bool last_macro_pos_valid_ = false;
    // Last 30 Hz local-planner turn state.  Macro maintenance runs before
    // the current 30 Hz plan on a 5 Hz boundary, so this one-tick-latched
    // value prevents a normal in-place turn from being mistaken for a
    // translational stall and prevents the 5 Hz guide from changing under
    // an active TURN_TO_TARGET maneuver.
    bool last_macro_turn_mode_ = false;
    // Last rolling-guide diagnostics (v7), stored so terminal states and the
    // final step output stay fully observable.
    double last_macro_route_progress_ = -1.0;
    double last_macro_guide_lookahead_ = 0.0;
    std::string last_macro_guide_update_reason_;
    // Once the blocker is passed this is LATCHED (never cleared by goal
    // changes); after that the macro uses a plain global route and never
    // fails the task on the fixed-side-route infeasibility.
    bool blocker_passed_latched_ = false;
    // FIXED physical homotopy reference (entry axis + blocker centre).
    HomotopyReference homotopy_ref_;

    // audit
    AuditFlags audit_;
    uint64_t macro_tick_event_ = 0;
    TransitionRecord last_transition_;
    // consecutive NO_SAFE_CANDIDATE ticks (UNKNOWN-driven; diagnostic only)
    uint32_t unknown_recovery_ticks_ = 0;
    uint32_t unknown_recovery_episode_count_ = 0;
};

}  // namespace il_2d_multiscale_debug
