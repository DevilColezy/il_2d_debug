#include "il_2d_multiscale_debug/hierarchical_expert_fsm.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace il_2d_multiscale_debug {

// ────────────────────────────────────────────────────────────────────
//  Construction / reset
// ────────────────────────────────────────────────────────────────────
HierarchicalExpertFsm::HierarchicalExpertFsm(const Params2D& p)
    : p_(p), local_planner_(p), corrector_(p), adapter_(p) {}

void HierarchicalExpertFsm::reset(const Task2D& task, uint64_t tick) {
    state_ = FsmState::DIRECT_LOCAL;
    tick_ = tick;
    consecutive_failures_ = 0;
    failure_start_tick_ = 0;
    unknown_recovery_ticks_ = 0;
    unknown_recovery_episode_count_ = 0;
    goal_revision_ = 0;
    mission_revision_ = 0;
    reentry_guard_ = 0;
    macro_tick_event_ = 0;
    last_transition_ = TransitionRecord{};
    audit_ = AuditFlags{};
    corrector_.reset();
    // Initial directive: PASS_THROUGH toward the task goal, event 0.
    directive_ = TargetCorrectionDirective{};
    directive_.valid = true;
    directive_.reason = "PASS_THROUGH";
    last_encoded_ = EncodedTargetInput{};
    directive_updated_ = false;
    last_delivered_event_ = 0;
    last_original_goal_ = task.goal;
    local_planner_.reset();
}

// ────────────────────────────────────────────────────────────────────
//  Helpers
// ────────────────────────────────────────────────────────────────────
bool HierarchicalExpertFsm::isTerminal(FsmState s) const {
    return s == FsmState::GOAL_REACHED || s == FsmState::COLLISION ||
           s == FsmState::TIMEOUT || s == FsmState::TASK_INVALID;
}

void HierarchicalExpertFsm::transition(FsmStepOutput& out, FsmState next,
                                       const std::string& reason) {
    if (next == state_) return;
    TransitionRecord rec;
    rec.happened = true;
    rec.prev = state_;
    rec.curr = next;
    rec.reason = reason;
    rec.tick = tick_;
    rec.failure_count_30hz = consecutive_failures_;
    rec.macro_stable_exit_count = 0;  // legacy: no macro exit counter
    rec.side = corrector_.lockedSide();
    rec.blocker_id = -1;  // legacy: no local blocker track
    rec.local_target_update_event = corrector_.directiveUpdateEvent();
    last_transition_ = rec;
    out.transition = rec;
    state_ = next;
}

LocalTarget HierarchicalExpertFsm::makeLocalTarget(
    const EncodedTargetInput& encoded) const {
    LocalTarget t;
    t.position = encoded.effective_target_world;
    t.valid = encoded.valid && encoded.effective_target_world_valid;
    // Compatibility-only field.  Keep it false so correction mode/side is
    // never smuggled into the 30 Hz planner's target contract.
    t.is_macro_guide = false;
    t.update_event = corrector_.directiveUpdateEvent();
    t.mission_revision = mission_revision_;
    t.normalized_distance = encoded.normalized_distance;
    return t;
}

bool HierarchicalExpertFsm::goalReached(const FsmInput& in) const {
    const double d = (in.state.position - in.task.goal).norm();
    const double v = in.state.velocity_world.norm();
    const double yr = std::fabs(in.state.yaw_rate);
    return d <= p_.task_goal_tolerance &&
           v < p_.vehicle_goal_stop_speed_mps &&
           yr <= p_.lp_turn_exit_max_yaw_rate;
}

void HierarchicalExpertFsm::updateFailureBookkeeping(
    const FsmStepOutput& out, const FsmInput& in) {
    // v9: DIAGNOSTIC ONLY.  These counters are published for the GUI / CSV
    // but are NEVER read by the 5 Hz VisibilityTargetCorrector (the 5 Hz
    // decision is purely observability-driven) and never trigger any
    // state transition.
    const bool blocked =
        out.local.failure_reason == FailureReason::BLOCKED_BY_OBSERVED_OBSTACLE;
    if (out.local.success || out.local.turn_mode) {
        consecutive_failures_ = 0;
        failure_start_tick_ = 0;
        unknown_recovery_ticks_ = 0;
    } else if (blocked) {
        if (consecutive_failures_ == 0) failure_start_tick_ = in.tick;
        ++consecutive_failures_;
        unknown_recovery_ticks_ = 0;
    } else if (out.local.failure_reason == FailureReason::NO_SAFE_CANDIDATE) {
        consecutive_failures_ = 0;
        failure_start_tick_ = 0;
        if (unknown_recovery_ticks_ == 0) ++unknown_recovery_episode_count_;
        ++unknown_recovery_ticks_;
    } else {
        consecutive_failures_ = 0;
        failure_start_tick_ = 0;
        unknown_recovery_ticks_ = 0;
    }
}

void HierarchicalExpertFsm::fillObservability(FsmStepOutput& out) const {
    // ── v9: correction + effective-target encoding ────────────────
    out.target_correction_type = static_cast<uint8_t>(directive_.type);
    out.target_correction_type_name =
        targetCorrectionTypeName(directive_.type);
    out.target_correction_active = corrector_.correctionActive();
    out.target_direction_token = directive_.direction_token;
    out.target_direction_x_body = last_encoded_.direction_body.x();
    out.target_direction_y_body = last_encoded_.direction_body.y();
    out.target_distance_normalized = last_encoded_.normalized_distance;
    out.effective_target_x = last_encoded_.effective_target_world.x();
    out.effective_target_y = last_encoded_.effective_target_world.y();
    out.effective_target_world_valid =
        last_encoded_.effective_target_world_valid;
    out.original_goal = last_original_goal_;
    const AvoidanceObservability& o = corrector_.lastObservability();
    out.observability_goal_inside_fov = o.goal_inside_fov;
    out.observability_direct_corridor_blocked = o.direct_corridor_blocked;
    out.observability_blocker_observed = o.blocker_observed;
    out.observability_left_bypass_visible = o.left_bypass_observable;
    out.observability_right_bypass_visible = o.right_bypass_observable;
    out.observability_local_avoidance_observable =
        o.local_avoidance_observable;
    out.observability_fov_boundary_truncated = o.fov_boundary_truncated;
    out.observability_unknown_occluded = o.unknown_occluded;
    out.observability_reason = o.reason;
    out.observability_left_score = o.left_score;
    out.observability_right_score = o.right_score;
    out.correction_enter_event = corrector_.correctionEnterEvent();
    out.correction_exit_event = corrector_.correctionExitEvent();
    out.correction_update_event = corrector_.correctionUpdateEvent();
    // LEFT/RIGHT latch (v9: the corrector's locked side).
    out.side = corrector_.lockedSide();
    // The legacy `evidence` carries the 5 Hz observability reason + side
    // scores so the existing side_reason / side_*_score message fields stay
    // informative (why does the 5 Hz layer think the FOV is unobservable?).
    out.evidence = SideEvidence{};
    out.evidence.reason = o.reason;
    out.evidence.left_score = o.left_score;
    out.evidence.right_score = o.right_score;
    // Body-frame supervision of the ACTUAL executed direction.
    out.relative_target_x_body = last_encoded_.direction_body.x();
    out.relative_target_y_body = last_encoded_.direction_body.y();
    out.target_bearing_deg = rad2deg(std::atan2(
        last_encoded_.direction_body.y(), last_encoded_.direction_body.x()));
    out.target_distance_m =
        last_encoded_.normalized_distance * p_.obs_range_m;

    // ── LEGACY macro fields: always invalid / empty (no decision use) ──
    out.macro_active = false;
    // NOTE: out.evidence is intentionally NOT reset here — it already
    // carries the 5 Hz observability reason + side scores (see above) so
    // the existing side_reason / side_*_score message fields stay useful.
    out.left_route = Route2D{};
    out.right_route = Route2D{};
    out.locked_route = Route2D{};
    out.blocker = BlockerInfo{};
    out.local_blocker = LocalBlockerEvidence{};
    out.blocker_association = BlockerAssociation::NONE;
    out.blocker_passed_latched = false;
    out.start_clearance_recovery_used = false;
    out.entry_vehicle_progress = -1.0;
    out.entry_blocker_progress = -1.0;
    out.entry_projection_dist = -1.0;
    out.entry_segment_index = -1;
    out.entry_progress_delta = 0.0;
    out.entry_progress_max_delta = 0.0;
    out.macro_route_progress = -1.0;
    out.macro_guide_lookahead = 0.0;
    out.macro_guide_update_reason = "";
    out.macro_no_progress_duration = 0.0;
    out.macro_used_local_history_only = true;
    out.macro_guide_inside_current_fov = false;
    out.macro_guide_endpoint_known_free = false;
    out.macro_guide_chord_known_free = false;
    out.macro_guide_min_observed_clearance =
        std::numeric_limits<double>::infinity();
    out.local_blocker_track_valid = false;
    out.local_blocker_track_id = -1;
    out.local_blocker_behind = false;
    out.local_goal_corridor_clear = false;
    out.local_leave_progress_m = 0.0;
    out.local_macro_route_valid = false;
    out.macro_stable_exit_count = 0;
    // ── Counters / audit / transition ──────────────────────────────
    out.consecutive_failures_30hz = consecutive_failures_;
    out.unknown_recovery_ticks = unknown_recovery_ticks_;
    out.unknown_recovery_active =
        static_cast<int>(unknown_recovery_ticks_) >=
        p_.macro_unknown_recovery_threshold_ticks;
    out.unknown_recovery_episode_count = unknown_recovery_episode_count_;
    out.macro_tick_event = macro_tick_event_;
    out.audit = audit_;
    out.transition = last_transition_;
    if (!out.local_target.valid) {
        out.local_target =
            makeLocalTarget(last_encoded_);
    }
}

void HierarchicalExpertFsm::forceCollision(FsmStepOutput& out) {
    if (isTerminal(state_)) return;
    TransitionRecord rec;
    rec.happened = true;
    rec.prev = state_;
    rec.curr = FsmState::COLLISION;
    rec.reason = "COLLISION_DETECTED";
    rec.tick = tick_;
    rec.failure_count_30hz = consecutive_failures_;
    rec.macro_stable_exit_count = 0;
    rec.side = corrector_.lockedSide();
    rec.blocker_id = -1;
    rec.local_target_update_event = corrector_.directiveUpdateEvent();
    last_transition_ = rec;
    out.transition = rec;
    out.state = FsmState::COLLISION;
    out.prev_state = rec.prev;
    state_ = FsmState::COLLISION;
    out.macro_tick_event = macro_tick_event_;
    out.local.success = false;
    out.local.vx_body = 0.0;
    out.local.vy_body = 0.0;
    out.local.yaw_rate = 0.0;
    // Terminal-state observability: expose the full correction / encoding /
    // observability picture so the GUI can inspect what led to the end.
    fillObservability(out);
}

// ────────────────────────────────────────────────────────────────────
//  Formal acceptance of a new final goal (5 Hz boundary only)
// ────────────────────────────────────────────────────────────────────
void HierarchicalExpertFsm::acceptNewGoal(const Vec2d& new_goal,
                                          uint64_t tick) {
    tick_ = tick;
    ++goal_revision_;
    // A formally accepted final-goal revision is a NEW MISSION CONTRACT:
    // mission_revision_ increments (the 30 Hz planner may reset its
    // per-mission memory).  v9: 5 Hz correction enter / refresh / exit
    // NEVER change mission_revision_ — only this function and reset() do.
    ++mission_revision_;
    // The 5 Hz corrector drops any active correction (locked side,
    // stability counters, re-entry guard) for the new goal, and the
    // directive update event is bumped so logs reflect the changed goal.
    corrector_.resetForNewGoal();
    directive_ = TargetCorrectionDirective{};
    directive_.valid = true;
    directive_.update_event = corrector_.bumpDirectiveEvent();
    directive_.reason = "NEW_FINAL_GOAL";
    (void)new_goal;
}

// ────────────────────────────────────────────────────────────────────
//  Main step
// ────────────────────────────────────────────────────────────────────
FsmStepOutput HierarchicalExpertFsm::step(const FsmInput& in) {
    FsmStepOutput out;
    tick_ = in.tick;
    out.prev_state = state_;
    out.state = state_;
    out.macro_tick_event = macro_tick_event_;
    last_original_goal_ = in.task.goal;

    // Terminal states are frozen but remain FULLY observable (correction
    // state, encoded target, observability, audit, transition).
    if (isTerminal(state_)) {
        out.local.success = false;
        out.local.vx_body = 0.0;
        out.local.vy_body = 0.0;
        out.local.yaw_rate = 0.0;
        fillObservability(out);
        return out;
    }

    // ── 5 Hz boundary: run the VisibilityTargetCorrector (exactly once
    //    per boundary, ZOH between boundaries).  The corrector reads ONLY
    //    state / original goal / current_patch / causal local history /
    //    its own memory — never
    //    the 30 Hz outcome. ──
    const bool is_5hz_tick = (in.tick % 6 == 0);
    if (is_5hz_tick) {
        ++macro_tick_event_;
        out.macro_tick_ran = true;
        directive_ = corrector_.update(in.state, in.task.goal,
                                       in.current_patch, in.history, in.tick);
    } else {
        out.macro_tick_ran = false;
    }
    // The "target updated this tick" flag reflects whether the directive
    // update event actually changed since the last delivered value (covers
    // both in-step corrector bumps AND goal-acceptance bumps done by
    // acceptNewGoal before this step).
    directive_updated_ = directive_.update_event != last_delivered_event_;
    last_delivered_event_ = directive_.update_event;

    // ── EffectiveTargetAdapter EVERY real 30 Hz tick: directive + live
    //    pose + original goal → EncodedTargetInput (body direction +
    //    normalized distance + effective world target). ──
    last_encoded_ = adapter_.encode(in.state, in.task.goal, directive_);

    // ── LocalTarget for the 30 Hz planner (world point from the adapter;
    //    update_event = directive update event; mission_revision unchanged
    //    by any 5 Hz activity). ──
    out.local_target_updated = directive_updated_;
    out.local_target = makeLocalTarget(last_encoded_);
    out.local_target.update_event = directive_.update_event;

    // ── 30 Hz local plan (merged HISTORY map only — the planner never
    //    sees current_patch or any privileged information). ──
    out.local = local_planner_.plan(in.state, in.history, out.local_target);

    // ── Terminal checks (override everything; goal reached judged on the
    //    ORIGINAL goal, never on the adapter's truncated point). ──
    if (in.collision) {
        transition(out, FsmState::COLLISION, "COLLISION_DETECTED");
    } else if (goalReached(in)) {
        transition(out, FsmState::GOAL_REACHED, "GOAL_REACHED");
    } else if (p_.task_episode_timeout_s > 0.0 &&
               static_cast<double>(in.tick) / 30.0 >=
                   p_.task_episode_timeout_s) {
        transition(out, FsmState::TIMEOUT, "EPISODE_TIMEOUT");
    } else {
        // ── 30 Hz failure bookkeeping (DIAGNOSTIC ONLY). ──
        updateFailureBookkeeping(out, in);
        // ── State machine: only the 30 Hz planner's own turn behaviour. ──
        switch (state_) {
            case FsmState::DIRECT_LOCAL:
                if (out.local.turn_mode) {
                    transition(out, FsmState::TURN_TO_TARGET,
                               "TARGET_OUTSIDE_FOV");
                }
                break;
            case FsmState::TURN_TO_TARGET:
                if (!out.local.turn_mode) {
                    transition(out, FsmState::DIRECT_LOCAL, "TARGET_IN_FOV");
                }
                break;
            default:
                break;
        }
    }

    // ── Sync display mirror of the corrector's re-entry guard. ──
    reentry_guard_ = corrector_.reentryGuardRemaining(in.tick);

    // ── Audit ──
    audit_.local_target_update_event = out.local_target.update_event;
    if (out.local.immediate_avoidance) ++audit_.immediate_avoidance_event;
    if (out.local.emergency_brake) ++audit_.emergency_brake_event;
    out.macro_tick_event = macro_tick_event_;
    out.state = state_;
    fillObservability(out);
    return out;
}

}  // namespace il_2d_multiscale_debug
