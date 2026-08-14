#include "il_2d_multiscale_debug/hierarchical_expert_fsm.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace il_2d_multiscale_debug {

// ────────────────────────────────────────────────────────────────────
//  Construction / reset
// ────────────────────────────────────────────────────────────────────
HierarchicalExpertFsm::HierarchicalExpertFsm(const Params2D& p)
    : p_(p), local_planner_(p), macro_expert_(p), route_planner_(p) {}

void HierarchicalExpertFsm::reset(const Task2D& task, uint64_t tick) {
    state_ = FsmState::DIRECT_LOCAL;
    tick_ = tick;
    side_ = SideSelection::NONE;
    evidence_ = SideEvidence{};
    left_route_ = Route2D{};
    right_route_ = Route2D{};
    locked_route_ = Route2D{};
    blocker_ = BlockerInfo{};
    local_evidence_ = LocalBlockerEvidence{};
    macro_target_ = LocalTarget{};
    consecutive_failures_ = 0;
    failure_start_tick_ = 0;
    macro_stable_exit_count_ = 0;
    reentry_guard_ = 0;
    effective_local_target_event_ = 0;
    // The initial goal is already the delivered local target (event 0).
    last_delivered_target_ = task.goal;
    last_delivered_valid_ = true;
    goal_revision_ = 0;
    mission_revision_ = 0;
    entry_reference_route_ = Route2D{};
    entry_reference_valid_ = false;
    entry_blocker_route_progress_ = -1.0;
    entry_vehicle_progress_ = -1.0;
    entry_vehicle_projection_dist_ = -1.0;
    entry_last_segment_index_ = -1;
    entry_progress_delta_ = 0.0;
    entry_progress_max_delta_ = 0.0;
    entry_progress_update_tick_ = std::numeric_limits<uint64_t>::max();
    // Rolling macro-guide no-progress bookkeeping (v7).
    macro_no_progress_duration_ = 0.0;
    last_macro_vehicle_pos_ = Vec2d(0.0, 0.0);
    last_macro_pos_valid_ = false;
    last_macro_turn_mode_ = false;
    macro_expert_.resetGuideState();
    last_macro_route_progress_ = -1.0;
    last_macro_guide_lookahead_ = 0.0;
    last_macro_guide_update_reason_.clear();
    blocker_passed_latched_ = false;
    homotopy_ref_ = HomotopyReference{};
    audit_ = AuditFlags{};
    macro_tick_event_ = 0;
    last_transition_ = TransitionRecord{};
    unknown_recovery_ticks_ = 0;
    unknown_recovery_episode_count_ = 0;
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
    rec.macro_stable_exit_count = macro_stable_exit_count_;
    rec.side = side_;
    rec.blocker_id = !blocker_.obstacle_ids.empty()
                         ? blocker_.obstacle_ids.front()
                         : -1;
    // Record the AUTHORITATIVE effective local-target event of this tick
    // (the FSM's own counter, final by the time the transition commits) —
    // never the possibly-unfilled out.local_target default value.
    rec.local_target_update_event = effective_local_target_event_;
    last_transition_ = rec;
    out.transition = rec;
    state_ = next;
}

void HierarchicalExpertFsm::fillMacroObservability(FsmStepOutput& out) const {
    out.side = side_;
    out.evidence = evidence_;
    out.left_route = left_route_;
    out.right_route = right_route_;
    out.locked_route = locked_route_;
    out.blocker = blocker_;
    out.local_blocker = local_evidence_;
    out.blocker_association = blocker_.association;
    out.blocker_passed_latched = blocker_passed_latched_;
    out.start_clearance_recovery_used =
        locked_route_.start_clearance_recovery_used;
    out.entry_vehicle_progress = entry_vehicle_progress_;
    out.entry_blocker_progress = entry_blocker_route_progress_;
    out.entry_projection_dist = entry_vehicle_projection_dist_;
    out.entry_segment_index = entry_last_segment_index_;
    out.entry_progress_delta = entry_progress_delta_;
    out.entry_progress_max_delta = entry_progress_max_delta_;
    out.macro_route_progress = last_macro_route_progress_;
    out.macro_guide_lookahead = last_macro_guide_lookahead_;
    out.macro_guide_update_reason = last_macro_guide_update_reason_;
    out.macro_no_progress_duration = macro_no_progress_duration_;
    out.consecutive_failures_30hz = consecutive_failures_;
    out.macro_stable_exit_count = macro_stable_exit_count_;
    out.unknown_recovery_ticks = unknown_recovery_ticks_;
    out.unknown_recovery_active =
        static_cast<int>(unknown_recovery_ticks_) >=
        p_.macro_unknown_recovery_threshold_ticks;
    out.unknown_recovery_episode_count = unknown_recovery_episode_count_;
    if (macro_target_.valid) {
        out.local_target = macro_target_;
    } else {
        out.local_target = makeLocalTarget(last_delivered_target_, false);
    }
    out.macro_active = state_ == FsmState::MACRO_GUIDANCE ||
                       state_ == FsmState::MACRO_EXIT_PENDING;
    out.audit = audit_;
    out.transition = last_transition_;
}

bool HierarchicalExpertFsm::goalReached(const FsmInput& in) const {
    const double d = (in.state.position - in.task.goal).norm();
    const double v = in.state.velocity_world.norm();
    const double yr = std::fabs(in.state.yaw_rate);
    return d <= p_.task_goal_tolerance &&
           v < p_.vehicle_goal_stop_speed_mps &&
           yr <= p_.lp_turn_exit_max_yaw_rate;
}

bool HierarchicalExpertFsm::issueLocalTargetEvent(const Vec2d& value) {
    // Value-change rule (documented in README): an event is issued ONLY
    // when the 30 Hz planner actually sees a different local-target value.
    const double tol = p_.macro_local_target_event_tolerance_m;
    if (last_delivered_valid_ &&
        (value - last_delivered_target_).norm() <= tol) {
        return false;
    }
    ++effective_local_target_event_;
    last_delivered_target_ = value;
    last_delivered_valid_ = true;
    return true;
}

LocalTarget HierarchicalExpertFsm::makeLocalTarget(const Vec2d& pos,
                                                   bool macro_guide) const {
    LocalTarget t;
    t.position = pos;
    t.valid = true;
    t.is_macro_guide = macro_guide;
    t.update_event = effective_local_target_event_;
    t.mission_revision = mission_revision_;
    return t;
}

bool HierarchicalExpertFsm::projectOntoRoute(const Route2D& route,
                                             const Vec2d& world,
                                             double& progress,
                                             double& lateral_dist) const {
    const auto& w = route.waypoints;
    if (w.size() < 2) {
        progress = 0.0;
        lateral_dist = 0.0;
        return false;
    }
    // Precompute total length; a degenerate (zero-length) route still
    // reports a valid projection onto its first waypoint.
    double total = 0.0;
    for (size_t i = 0; i + 1 < w.size(); ++i) {
        total += (w[i + 1] - w[i]).norm();
    }
    if (total <= 1e-9) {
        progress = 0.0;
        lateral_dist = (w.front() - world).norm();
        return true;
    }

    // Continuous segment-wise projection: for every segment the closest
    // point is the clamped perpendicular foot, so progress is a true arc
    // length (not merely the nearest-waypoint arc length).
    double best_sq = std::numeric_limits<double>::infinity();
    double best_prog = 0.0;
    double best_lat = 0.0;
    double acc = 0.0;
    for (size_t i = 0; i + 1 < w.size(); ++i) {
        const Vec2d a = w[i];
        const Vec2d b = w[i + 1];
        const Vec2d ab = b - a;
        const double len2 = ab.squaredNorm();
        double t = 0.0;
        if (len2 > 1e-12) {
            t = std::clamp((world - a).dot(ab) / len2, 0.0, 1.0);
        }
        const Vec2d proj = a + t * ab;
        const double d2 = (proj - world).squaredNorm();
        if (d2 < best_sq) {
            best_sq = d2;
            best_prog = acc + t * std::sqrt(len2);
            best_lat = std::sqrt(d2);
        }
        acc += std::sqrt(len2);
    }
    progress = best_prog;
    lateral_dist = best_lat;
    return true;
}

void HierarchicalExpertFsm::updateEntryProgress(const Vec2d& pos) {
    if (!entry_reference_valid_) return;
    if (entry_progress_update_tick_ == tick_) return;
    entry_progress_update_tick_ = tick_;
    const auto& w = entry_reference_route_.waypoints;
    if (w.size() < 2) return;

    // Per-segment lengths + start arc lengths (used to bound the search).
    const int nseg = static_cast<int>(w.size()) - 1;
    std::vector<double> seg_len(nseg, 0.0);
    std::vector<double> seg_start(nseg + 1, 0.0);
    for (int i = 0; i < nseg; ++i) {
        seg_len[i] = (w[i + 1] - w[i]).norm();
        seg_start[i + 1] = seg_start[i] + seg_len[i];
    }
    const double total = seg_start[nseg];
    if (total <= 1e-9) return;

    // Arc-length search window: forward progress in one 5 Hz interval is
    // bounded by max_speed × (6/30) plus a tolerance; backward window
    // tolerates stops.  This makes a full-route nearest-point + max()
    // jump to a far-away similar segment impossible.
    const double max_step =
        p_.lp_max_speed * (6.0 / 30.0) + p_.macro_progress_forward_tolerance_m;
    entry_progress_delta_ = 0.0;
    entry_progress_max_delta_ = max_step;
    const double base =
        (entry_vehicle_progress_ >= 0.0) ? entry_vehicle_progress_ : 0.0;
    const double lo = std::max(0.0, base - p_.macro_progress_back_window_m);
    const double hi = std::min(total, base + max_step);

    // Segment window: every segment overlapping the arc-length window
    // (NOT a fixed ±2 around the last match — the vehicle can advance
    // several route segments per 5 Hz interval).  The anti-jump property
    // comes from the FORWARD-CAPPED arc-length window itself: a far-away
    // similar segment can never be reached in one interval.
    int lo_seg = 0, hi_seg = nseg - 1;
    while (lo_seg < nseg - 1 && seg_start[lo_seg + 1] < lo - 1e-9) ++lo_seg;
    while (hi_seg > 0 && seg_start[hi_seg] > hi + 1e-9) --hi_seg;

    double best_lat = std::numeric_limits<double>::infinity();
    double best_prog = -1.0;
    int best_seg = -1;
    for (int i = lo_seg; i <= hi_seg; ++i) {
        if (seg_start[i + 1] < lo - 1e-9) continue;
        if (seg_start[i] > hi + 1e-9) continue;
        const Vec2d a = w[i], b = w[i + 1];
        const Vec2d ab = b - a;
        const double len2 = ab.squaredNorm();
        double t = 0.0;
        if (len2 > 1e-12) {
            t = std::clamp((pos - a).dot(ab) / len2, 0.0, 1.0);
        }
        const double prog = seg_start[i] + t * seg_len[i];
        if (prog < lo - 1e-9 || prog > hi + 1e-9) continue;
        const Vec2d proj = a + t * ab;
        const double lat = (proj - pos).norm();
        if (lat < best_lat) {
            best_lat = lat;
            best_prog = prog;
            best_seg = i;
        }
    }
    if (best_seg < 0) return;  // nothing in the window → keep old progress

    // Always track the local match (so the window follows the vehicle),
    // but only advance PROGRESS monotonically.
    entry_vehicle_projection_dist_ = best_lat;
    entry_last_segment_index_ = best_seg;
    if (best_lat <= p_.macro_blocker_projection_max_dist_m &&
        best_prog > entry_vehicle_progress_) {
        const double previous = std::max(0.0, entry_vehicle_progress_);
        entry_vehicle_progress_ = best_prog;
        entry_progress_delta_ = entry_vehicle_progress_ - previous;
    }

    // LATCH the blocker-passed condition once reached — never un-latch.
    if (entry_blocker_route_progress_ >= 0.0 &&
        entry_vehicle_progress_ > entry_blocker_route_progress_ +
                                      p_.macro_blocker_clear_dist_m) {
        blocker_passed_latched_ = true;
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
    rec.macro_stable_exit_count = macro_stable_exit_count_;
    rec.side = side_;
    rec.blocker_id = !blocker_.obstacle_ids.empty()
                         ? blocker_.obstacle_ids.front()
                         : -1;
    rec.local_target_update_event = effective_local_target_event_;
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
    // Terminal-state observability: expose the full macro picture (side,
    // evidence, routes, blockers, association, latched progress, counters)
    // so the GUI can inspect what led to the collision.
    fillMacroObservability(out);
}

bool HierarchicalExpertFsm::canEnterMacro(const FsmStepOutput& out,
                                          const FsmInput& in) const {
    // 1) target already inside the 30 Hz FOV (not turning),
    // 2) failure is a real observed blockage,
    // 3) velocity can brake safely,
    // 4) not a single-shot sample failure (already accumulated by caller),
    // 5) re-entry hysteresis guard elapsed.
    if (out.local.turn_mode) return false;
    if (out.local.failure_reason != FailureReason::BLOCKED_BY_OBSERVED_OBSTACLE)
        return false;
    if (!local_planner_.canBrakeSafely(in.state, in.history)) return false;
    if (reentry_guard_ > 0) return false;
    return true;
}

// ────────────────────────────────────────────────────────────────────
//  Macro decisions (called ONLY on tick % 6 == 0)
// ────────────────────────────────────────────────────────────────────
void HierarchicalExpertFsm::runMacroDecision(FsmStepOutput& out,
                                             const FsmInput& in) {
    // LOCAL blocker EVIDENCE from the CURRENT INSTANTANEOUS FOV PATCH
    // only (no truth, no history map).
    local_evidence_ =
        macro_expert_.identifyBlocker(in.state, in.task.goal, in.current_patch);
    // PRIVILEGED blocker: deterministic cell-support association to truth
    // cylinders.  NO_MATCH / AMBIGUOUS_MATCH are hard failures — never a
    // silent plain-A* fallback.
    blocker_ = macro_expert_.resolvePrivilegedBlocker(local_evidence_,
                                                      in.scene);
    out.blocker = blocker_;
    out.local_blocker = local_evidence_;
    out.blocker_association = blocker_.association;
    if (blocker_.association == BlockerAssociation::NO_MATCH ||
        blocker_.association == BlockerAssociation::AMBIGUOUS_MATCH) {
        transition(out, FsmState::TASK_INVALID,
                   blocker_.association == BlockerAssociation::NO_MATCH
                       ? "TASK_INVALID_BLOCKER_ASSOCIATION (NO_MATCH)"
                       : "TASK_INVALID_BLOCKER_ASSOCIATION (AMBIGUOUS_MATCH)");
        out.side = side_;
        out.evidence = evidence_;
        return;
    }

    // Side selection STRICTLY from visible evidence of the CURRENT FOV
    // PATCH (never the history map).
    evidence_ = macro_expert_.selectSideFromVisibleEvidence(
        in.state, in.current_patch, in.task.goal);
    side_ = evidence_.selection;
    audit_.side_selected_from_visible_evidence = evidence_.from_visible_evidence;
    audit_.side_ambiguous_defaulted_right = evidence_.ambiguous_defaulted_right;
    audit_.side_selected_using_current_patch = true;

    // Capture the FIXED physical homotopy reference (entry axis + blocker
    // centre + side).  From now on LEFT/RIGHT are interpreted only
    // relative to this reference until the blocker is passed.
    homotopy_ref_.valid = true;
    const Vec2d axis = in.task.goal - in.state.position;
    const double axis_len = std::max(1e-6, axis.norm());
    homotopy_ref_.entry_axis = axis / axis_len;
    homotopy_ref_.entry_blocker_center = blocker_.center;
    homotopy_ref_.side = side_;
    homotopy_ref_.side_normal_sign =
        (side_ == SideSelection::LEFT) ? 1.0 : -1.0;

    // Build BOTH privileged routes for display, then keep the selected
    // one — all geometry relative to the FIXED homotopy reference.
    Route2D left, right, locked;
    macro_expert_.buildRoutes(in.scene, in.esdf, in.state, in.task.goal,
                              blocker_, side_, &homotopy_ref_, left, right,
                              locked);
    left_route_ = left;
    right_route_ = right;
    locked_route_ = locked;
    audit_.macro_used_privileged_esdf = true;

    if (!locked.valid) {
        // The causally-chosen side is globally infeasible → NEVER silently
        // switch to the other side.  Mark the task invalid.
        transition(out, FsmState::TASK_INVALID, "TASK_INVALID_FOR_CAUSAL_RULE");
        out.evidence = evidence_;
        out.side = side_;
        return;
    }

    // Guidance target along the locked route (ZOH until next 5 Hz).
    // ONE effective local-target event, only when the value actually
    // changes (entering macro usually hands a different target).  The
    // rolling-guide bookkeeping (monotonic progress + hysteresis) is
    // reset on entry so the first guide is selected fresh.
    macro_expert_.resetGuideState();
    macro_no_progress_duration_ = 0.0;
    last_macro_pos_valid_ = false;
    last_macro_turn_mode_ = false;
    double guide_lookahead = 0.0;
    double guide_progress = -1.0;
    std::string guide_reason;
    macro_target_ = macro_expert_.guidanceTarget(
        in.state, locked_route_, in.task.goal, in.esdf, in.tick,
        /*force_advance=*/false,
        guide_lookahead, guide_progress, guide_reason);
    out.macro_route_progress = guide_progress;
    out.macro_guide_lookahead = guide_lookahead;
    out.macro_guide_update_reason = guide_reason;
    last_macro_route_progress_ = guide_progress;
    last_macro_guide_lookahead_ = guide_lookahead;
    last_macro_guide_update_reason_ = guide_reason;
    out.local_target_updated = issueLocalTargetEvent(macro_target_.position);
    macro_target_.update_event = effective_local_target_event_;
    macro_target_.mission_revision = mission_revision_;
    audit_.local_target_update_event = effective_local_target_event_;

    // Save the FIXED macro-entry references: the entry locked route, the
    // blocker arc-length along it, and reset the monotonic vehicle
    // progress / latched flags.  The exit check compares the vehicle's
    // windowed monotonic progress along THIS fixed route — never against
    // the ever-changing goal direction.
    entry_reference_route_ = locked_route_;
    entry_reference_valid_ = true;
    entry_vehicle_progress_ = -1.0;
    entry_vehicle_projection_dist_ = -1.0;
    entry_last_segment_index_ = -1;
    entry_progress_delta_ = 0.0;
    entry_progress_max_delta_ = 0.0;
    entry_progress_update_tick_ = std::numeric_limits<uint64_t>::max();
    blocker_passed_latched_ = false;
    entry_blocker_route_progress_ = -1.0;
    if (blocker_.found) {
        double prog = 0.0, lat = 0.0;
        if (projectOntoRoute(entry_reference_route_, blocker_.center, prog,
                             lat)) {
            entry_blocker_route_progress_ = prog;
        }
    }

    transition(out, FsmState::MACRO_GUIDANCE, "MACRO_ENTER");
    // Entering macro mode is a mission-revision change: the 30 Hz planner
    // may reset its per-mission memory (command continuity + stall window)
    // — explicitly allowed by the reset contract.  Re-tag the guide target.
    ++mission_revision_;
    macro_target_.mission_revision = mission_revision_;
    ++audit_.macro_enter_event;
    out.evidence = evidence_;
    out.side = side_;
    out.blocker = blocker_;
    out.local_blocker = local_evidence_;
    out.blocker_association = blocker_.association;
}

void HierarchicalExpertFsm::runMacroGuidance(FsmStepOutput& out,
                                             const FsmInput& in) {
    // 1) Update the fixed-entry-route progress FIRST (windowed, monotonic,
    //    latched).  The blocker-passed flag only ever moves 0 → 1.
    updateEntryProgress(in.state.position);

    // Macro maintenance runs before the current 30 Hz local plan on a
    // boundary tick.  Keep a healthy guide stable while the local planner is
    // intentionally turning toward it.
    const bool hold_guide_for_turn =
        last_macro_turn_mode_ && macro_target_.valid;
    const LocalTarget previous_macro_target = macro_target_;

    // ── v7: macro no-progress bookkeeping + guide re-advance ───────
    // Even when the vehicle does not translate, the 5 Hz expert must keep
    // re-evaluating the guide.  After a sustained no-progress interval the
    // guide hysteresis is dropped so a stale / foot-level guide can never be
    // held, and the guide is re-selected from the freshly rebuilt route.
    const double dt_5hz = 6.0 / 30.0;
    const double displacement = last_macro_pos_valid_
                                    ? (in.state.position -
                                       last_macro_vehicle_pos_).norm()
                                    : 0.0;
    if (hold_guide_for_turn) {
        // In-place heading convergence is expected progress, not a stall.
        macro_no_progress_duration_ = 0.0;
    } else if (displacement < p_.macro_no_progress_threshold_m) {
        macro_no_progress_duration_ += dt_5hz;
    } else {
        macro_no_progress_duration_ = 0.0;
    }
    last_macro_vehicle_pos_ = in.state.position;
    last_macro_pos_valid_ = true;
    out.macro_no_progress_duration = macro_no_progress_duration_;
    const bool force_guide_refresh =
        !hold_guide_for_turn &&
        macro_no_progress_duration_ >=
            p_.macro_no_progress_duration_threshold_s;
    double guide_lookahead = 0.0;
    double guide_progress = -1.0;
    std::string guide_reason;
    if (blocker_passed_latched_) {
        // Blocker passed: NO forced gateways around the original blocker,
        // NO homotopy constraint, and the fixed-side route being locally
        // infeasible must NEVER fail the task.  Use a plain global-ESDF
        // route to keep producing a guidance target.
        locked_route_ =
            macro_expert_.buildPlainRoute(in.esdf, in.state, in.task.goal);
        left_route_ = Route2D{};
        right_route_ = Route2D{};
        if (!hold_guide_for_turn) {
            macro_target_ = macro_expert_.guidanceTarget(
                in.state, locked_route_, in.task.goal, in.esdf, in.tick,
                force_guide_refresh, guide_lookahead, guide_progress,
                guide_reason);
        }
        if (!macro_target_.valid) macro_target_.position = in.task.goal;
    } else {
        // Blocker NOT passed yet: refresh the fixed-side constrained route
        // (gateways / homotopy relative to the FIXED homotopy reference).
        Route2D left, right, locked;
        macro_expert_.buildRoutes(in.scene, in.esdf, in.state, in.task.goal,
                                  blocker_, side_, &homotopy_ref_, left, right,
                                  locked);
        left_route_ = left;
        right_route_ = right;
        locked_route_ = locked;
        if (!locked.valid) {
            // The locked side became infeasible → TASK_INVALID, never
            // switch (only while the blocker is not yet passed).
            transition(out, FsmState::TASK_INVALID,
                       "TASK_INVALID_FOR_CAUSAL_RULE");
            out.evidence = evidence_;
            out.side = side_;
            out.blocker = blocker_;
            out.local_blocker = local_evidence_;
            out.blocker_association = blocker_.association;
            return;
        }
        if (!hold_guide_for_turn) {
            macro_target_ = macro_expert_.guidanceTarget(
                in.state, locked_route_, in.task.goal, in.esdf, in.tick,
                force_guide_refresh, guide_lookahead, guide_progress,
                guide_reason);
        }
    }

    if (hold_guide_for_turn) {
        guide_progress = last_macro_route_progress_;
        guide_lookahead = last_macro_guide_lookahead_;
        guide_reason = "turning_hold";
    } else if (previous_macro_target.valid && macro_target_.valid) {
        // Do not replace a healthy forward guide with a large backward jump
        // caused by rebuilding/projection of the rolling route.  Such a jump
        // makes the 30 Hz planner alternate between translation and a fresh
        // TURN_TO_TARGET cycle.
        const Vec2d delta =
            macro_target_.position - previous_macro_target.position;
        const Vec2d to_candidate = macro_target_.position - in.state.position;
        const double candidate_bearing =
            std::atan2(to_candidate.y(), to_candidate.x()) - in.state.yaw;
        if (delta.norm() >
                std::max(1.5, p_.macro_guide_min_distance_m) &&
            std::fabs(wrapAngle(candidate_bearing)) > 0.5 * M_PI) {
            macro_target_ = previous_macro_target;
            guide_progress = last_macro_route_progress_;
            guide_lookahead = last_macro_guide_lookahead_;
            guide_reason = "backward_jump_hold";
        }
    }

    out.macro_route_progress = guide_progress;
    out.macro_guide_lookahead = guide_lookahead;
    out.macro_guide_update_reason = guide_reason;
    last_macro_route_progress_ = guide_progress;
    last_macro_guide_lookahead_ = guide_lookahead;
    last_macro_guide_update_reason_ = guide_reason;
    // Treat forced advance as a recovery pulse.  If the vehicle remains
    // stationary, another real forward advance is attempted after the next
    // complete no-progress interval rather than latching this state forever.
    if (force_guide_refresh) macro_no_progress_duration_ = 0.0;

    out.evidence = evidence_;
    out.side = side_;
    out.blocker = blocker_;
    out.local_blocker = local_evidence_;
    out.blocker_association = blocker_.association;
    out.blocker_passed_latched = blocker_passed_latched_;
    out.entry_vehicle_progress = entry_vehicle_progress_;
    out.entry_blocker_progress = entry_blocker_route_progress_;
    out.entry_projection_dist = entry_vehicle_projection_dist_;
    out.entry_segment_index = entry_last_segment_index_;
    out.entry_progress_delta = entry_progress_delta_;
    out.entry_progress_max_delta = entry_progress_max_delta_;

    // Exit-condition evaluation (needs a NON-mutating 30 Hz preview toward
    // the final goal using only the HISTORY map — the planner's own view).
    // Preview the exact post-handoff contract.  A real macro exit increments
    // mission_revision_, so this preview must not inherit command-continuity
    // costs from the currently active macro guide.
    LocalTarget exit_target = makeLocalTarget(in.task.goal, false);
    exit_target.mission_revision = mission_revision_ + 1;
    const PreviewResult preview =
        local_planner_.previewPlan(in.state, in.history, exit_target);
    const MacroExitCheck ec = macro_expert_.checkExitConditions(
        in.state, in.task.goal, preview, blocker_passed_latched_);

    if (ec.all()) {
        ++macro_stable_exit_count_;
        if (macro_stable_exit_count_ >=
            static_cast<uint32_t>(p_.macro_exit_stable_ticks)) {
            // Exit: cancel the macro target, local target → final goal.
            // The event (if the value changed) is issued by step()'s
            // direct-mode branch on this same tick — exactly one event.
            macro_target_ = LocalTarget{};
            out.local_target_updated =
                out.local_target_updated || issueLocalTargetEvent(in.task.goal);
            audit_.local_target_update_event = effective_local_target_event_;
            transition(out, FsmState::DIRECT_LOCAL, "MACRO_EXIT");
            // Exiting macro mode is a mission-revision change (the 30 Hz
            // planner may reset its per-mission memory — allowed).
            ++mission_revision_;
            // Clear the macro no-progress bookkeeping for the next episode.
            macro_no_progress_duration_ = 0.0;
            last_macro_pos_valid_ = false;
            macro_expert_.resetGuideState();
            ++audit_.macro_exit_event;
            reentry_guard_ = p_.macro_reentry_guard_ticks;
            macro_stable_exit_count_ = 0;
            return;
        }
        out.local_target_updated = issueLocalTargetEvent(macro_target_.position);
        macro_target_.update_event = effective_local_target_event_;
        macro_target_.mission_revision = mission_revision_;
        audit_.local_target_update_event = effective_local_target_event_;
        transition(out, FsmState::MACRO_EXIT_PENDING, "MACRO_EXIT_PENDING");
    } else {
        out.local_target_updated = issueLocalTargetEvent(macro_target_.position);
        macro_target_.update_event = effective_local_target_event_;
        macro_target_.mission_revision = mission_revision_;
        audit_.local_target_update_event = effective_local_target_event_;
        macro_stable_exit_count_ = 0;
        if (state_ == FsmState::MACRO_EXIT_PENDING) {
            transition(out, FsmState::MACRO_GUIDANCE, "MACRO_EXIT_CONDITION_LOST");
        }
    }

}

// ────────────────────────────────────────────────────────────────────
//  Formal acceptance of a new final goal (5 Hz boundary only)
// ────────────────────────────────────────────────────────────────────
void HierarchicalExpertFsm::acceptNewGoal(const Scene2D& scene,
                                          const TruthEsdf2D& esdf,
                                          const VehicleState2D& state,
                                          const Vec2d& new_goal,
                                          uint64_t tick) {
    // Final goal formally accepted: monotonic revision counter for the
    // visualization layer.  The effective local-target event is NOT issued
    // here — step()'s direct-mode branch / runMacroGuidance issues exactly
    // ONE event when the 30 Hz-visible target VALUE changes.
    tick_ = tick;
    ++goal_revision_;
    // A formally accepted final-goal revision is a new mission contract,
    // even if the new position is close to the old one.  Continuous rolling
    // LocalTarget updates do not call this function and therefore preserve
    // planner memory.
    ++mission_revision_;
    if (state_ == FsmState::MACRO_GUIDANCE || state_ == FsmState::MACRO_EXIT_PENDING) {
        // Acceptance precedes runMacroGuidance on this 5 Hz boundary.
        // Refresh the fixed-route latch before rebuilding any constrained
        // route for the new goal.
        updateEntryProgress(state.position);
        // Keep the locked side (causal rule), the CURRENT blocker, the
        // FIXED homotopy reference and the blocker-passed latch — none of
        // them are ever reset by a goal change.  The entry reference
        // (route + progress) is also preserved.
        if (blocker_passed_latched_) {
            // Blocker already passed: plain global route for the new goal;
            // never TASK_INVALID on the old fixed-side route.
            locked_route_ =
                macro_expert_.buildPlainRoute(esdf, state, new_goal);
            left_route_ = Route2D{};
            right_route_ = Route2D{};
            return;  // runMacroGuidance recomputes the macro target
        }
        Route2D left, right, locked;
        macro_expert_.buildRoutes(scene, esdf, state, new_goal, blocker_,
                                  side_, &homotopy_ref_, left, right, locked);
        left_route_ = left;
        right_route_ = right;
        if (!locked.valid) {
            const FsmState prev = state_;  // save BEFORE switching
            state_ = FsmState::TASK_INVALID;
            TransitionRecord rec;
            rec.happened = true;
            rec.prev = prev;
            rec.curr = FsmState::TASK_INVALID;
            rec.reason = "TASK_INVALID_FOR_CAUSAL_RULE (goal changed)";
            rec.tick = tick_;
            rec.failure_count_30hz = consecutive_failures_;
            rec.macro_stable_exit_count = macro_stable_exit_count_;
            rec.side = side_;
            rec.blocker_id = !blocker_.obstacle_ids.empty()
                                 ? blocker_.obstacle_ids.front()
                                 : -1;
            rec.local_target_update_event = effective_local_target_event_;
            last_transition_ = rec;
            return;
        }
        locked_route_ = locked;
        // The same boundary's runMacroGuidance() recomputes the macro
        // target from the refreshed route and issues the single event.
    }
    // Direct mode: the boundary tick's direct-mode branch handles it.
    (void)esdf;
}

// ────────────────────────────────────────────────────────────────────
//  Main step
// ────────────────────────────────────────────────────────────────────
FsmStepOutput HierarchicalExpertFsm::step(const FsmInput& in) {
    FsmStepOutput out;
    tick_ = in.tick;
    out.prev_state = state_;
    out.state = state_;

    // Terminal states are frozen but remain FULLY observable: side,
    // evidence, routes, blocker, local blocker, association, latched
    // progress, local target, counters, audit and the last transition are
    // all still published so the GUI can inspect what caused the end.
    if (isTerminal(state_)) {
        out.macro_tick_event = macro_tick_event_;
        out.local.success = false;
        out.local.vx_body = 0.0;
        out.local.vy_body = 0.0;
        out.local.yaw_rate = 0.0;
        fillMacroObservability(out);
        return out;
    }

    // Macro re-entry guard: SATURATING decrement on EVERY non-terminal
    // 30 Hz tick (unit = 30 Hz ticks, never below 0), not only at 5 Hz
    // boundaries.
    if (reentry_guard_ > 0) --reentry_guard_;

    const bool is_macro_tick = (in.tick % 6 == 0);
    bool macro_ran = false;  // the macro expert runs at most once per boundary
    if (is_macro_tick) {
        ++macro_tick_event_;
        out.macro_tick_ran = true;
    }

    // ── 5 Hz macro maintenance BEFORE the local plan on boundary ticks
    //    (only for an already-active macro; triggering is handled below). ──
    if (is_macro_tick) {
        if (state_ == FsmState::MACRO_SELECT_SIDE) {
            macro_ran = true;
            runMacroDecision(out, in);
            if (isTerminal(state_)) {
                out.state = state_;
                out.macro_tick_event = macro_tick_event_;
                fillMacroObservability(out);
                return out;
            }
        } else if (state_ == FsmState::MACRO_GUIDANCE ||
                   state_ == FsmState::MACRO_EXIT_PENDING) {
            macro_ran = true;
            runMacroGuidance(out, in);
            if (isTerminal(state_)) {
                out.state = state_;
                out.macro_tick_event = macro_tick_event_;
                fillMacroObservability(out);
                return out;
            }
        }
    }

    // ── Local target (zero-order held macro target, else final goal). ──
    if (state_ == FsmState::MACRO_GUIDANCE || state_ == FsmState::MACRO_EXIT_PENDING) {
        out.local_target = macro_target_;
    } else {
        // The event and value must arrive at the 30 Hz planner together.
        out.local_target_updated = issueLocalTargetEvent(in.task.goal);
        out.local_target = makeLocalTarget(in.task.goal, false);
    }

    // ── 30 Hz local plan (the ONE plan that drives failure counting). ──
    // Uses the merged HISTORY map — the planner never sees current_patch
    // or any privileged information.
    out.local = local_planner_.plan(in.state, in.history, out.local_target);
    if (state_ == FsmState::MACRO_GUIDANCE ||
        state_ == FsmState::MACRO_EXIT_PENDING) {
        last_macro_turn_mode_ = out.local.turn_mode;
    } else {
        last_macro_turn_mode_ = false;
    }

    // ── Terminal checks (override everything). ──
    if (in.collision) {
        transition(out, FsmState::COLLISION, "COLLISION_DETECTED");
    } else if (goalReached(in)) {
        transition(out, FsmState::GOAL_REACHED, "GOAL_REACHED");
    } else if (p_.task_episode_timeout_s > 0.0 &&
               static_cast<double>(in.tick) / 30.0 >=
                   p_.task_episode_timeout_s) {
        transition(out, FsmState::TIMEOUT, "EPISODE_TIMEOUT");
    } else {
        // ── 30 Hz failure bookkeeping (BEFORE state transitions). ──
        const bool blocked =
            out.local.failure_reason == FailureReason::BLOCKED_BY_OBSERVED_OBSTACLE;
        const bool in_macro = state_ == FsmState::MACRO_GUIDANCE ||
                              state_ == FsmState::MACRO_EXIT_PENDING ||
                              state_ == FsmState::MACRO_SELECT_SIDE;
        if (out.local.success || out.local.turn_mode) {
            consecutive_failures_ = 0;
            failure_start_tick_ = 0;
            unknown_recovery_ticks_ = 0;
        } else if (blocked) {
            if (consecutive_failures_ == 0) failure_start_tick_ = in.tick;
            ++consecutive_failures_;
            // Real observed blockage is NOT an UNKNOWN-recovery situation.
            unknown_recovery_ticks_ = 0;
        } else if (out.local.failure_reason ==
                   FailureReason::NO_SAFE_CANDIDATE) {
            // NO_SAFE_CANDIDATE (UNKNOWN/FOV/sample-set driven) NEVER counts
            // towards the macro budget and is NEVER mislabelled as real
            // blockage.  It is tracked only as LOCAL_UNKNOWN_RECOVERY.
            consecutive_failures_ = 0;
            failure_start_tick_ = 0;
            if (in_macro) {
                unknown_recovery_ticks_ = 0;
            } else {
                if (unknown_recovery_ticks_ == 0) {
                    ++unknown_recovery_episode_count_;
                }
                ++unknown_recovery_ticks_;
            }
        } else {
            // Other failures (STALLED_WITHOUT_PROGRESS, an invalid target
            // contract, ...) are neither observed blockage nor UNKNOWN
            // recovery.  STALLED_WITHOUT_PROGRESS is a 30 Hz-internal stall
            // WITHOUT observed-blockage evidence: it must never accumulate
            // the macro budget and never be masked by the 5 Hz privileged
            // expert (the planner reports failure_reason BLOCKED when real
            // evidence exists).  Keep both counters clear so the regression
            // is visible in planner_status instead.
            consecutive_failures_ = 0;
            failure_start_tick_ = 0;
            unknown_recovery_ticks_ = 0;
        }
        const double dur = consecutive_failures_ > 0
                               ? static_cast<double>(in.tick -
                                                    failure_start_tick_ + 1) /
                                     30.0
                               : 0.0;
        const bool threshold_reached =
            consecutive_failures_ > 0 && dur >= p_.macro_local_failure_duration_s;

        // ── State machine (non-macro transitions). ──
        switch (state_) {
            case FsmState::DIRECT_LOCAL: {
                if (out.local.turn_mode) {
                    transition(out, FsmState::TURN_TO_TARGET, "TARGET_OUTSIDE_FOV");
                } else if (threshold_reached) {
                    transition(out, FsmState::LOCAL_BLOCKED_PENDING,
                               "LOCAL_BLOCKED_PENDING");
                }
                break;
            }
            case FsmState::TURN_TO_TARGET: {
                if (!out.local.turn_mode) {
                    transition(out, FsmState::DIRECT_LOCAL, "TARGET_IN_FOV");
                }
                break;
            }
            case FsmState::LOCAL_BLOCKED_PENDING: {
                if (out.local.success) {
                    transition(out, FsmState::DIRECT_LOCAL, "LOCAL_UNBLOCKED");
                }
                break;
            }
            case FsmState::MACRO_SELECT_SIDE:
            case FsmState::MACRO_GUIDANCE:
            case FsmState::MACRO_EXIT_PENDING: {
                // Guidance handled above; nothing else needed here.
                break;
            }
            default:
                break;
        }

        // ── Unified same-tick macro trigger. ─────────────────────────
        // Works regardless of whether this tick started in DIRECT_LOCAL or
        // LOCAL_BLOCKED_PENDING: if the continuous local failure reaches
        // its threshold exactly ON this 5 Hz boundary (and every gate
        // holds), the full chain runs in this same tick:
        //   LOCAL_BLOCKED_PENDING → MACRO_SELECT_SIDE → runMacroDecision()
        //   → MACRO_GUIDANCE
        // The macro expert executes at most once per boundary (macro_ran).
        if (threshold_reached && is_macro_tick && !macro_ran &&
            canEnterMacro(out, in)) {
            macro_ran = true;
            if (state_ != FsmState::MACRO_SELECT_SIDE) {
                transition(out, FsmState::MACRO_SELECT_SIDE, "MACRO_TRIGGERED");
            }
            runMacroDecision(out, in);
            if (state_ == FsmState::MACRO_GUIDANCE) {
                // Re-plan ONCE toward the fresh macro target (ZOH applies
                // from this tick onward).  This second plan does NOT
                // re-accumulate failures (consecutive_failures_ is FSM
                // state) and does NOT re-issue macro events (the audit was
                // set exactly once inside runMacroDecision).
                out.local_target = macro_target_;
                out.local = local_planner_.plan(in.state, in.history,
                                                out.local_target);
                last_macro_turn_mode_ = out.local.turn_mode;
                audit_.local_target_update_event = out.local_target.update_event;
            }
        }
    }

    // ── Direct-mode local-target event (value-change rule). ──────────
    // Runs on every non-macro tick (including the macro-exit tick when the
    // final goal is restored); issues at most one event when the value
    // actually changed.
    if (!(state_ == FsmState::MACRO_GUIDANCE ||
          state_ == FsmState::MACRO_EXIT_PENDING)) {
        const bool changed = issueLocalTargetEvent(in.task.goal);
        out.local_target_updated = out.local_target_updated || changed;
        out.local_target.update_event = effective_local_target_event_;
        out.local_target.mission_revision = mission_revision_;
    }

    out.state = state_;
    out.macro_active = state_ == FsmState::MACRO_GUIDANCE ||
                       state_ == FsmState::MACRO_EXIT_PENDING ||
                       state_ == FsmState::MACRO_SELECT_SIDE;
    out.side = side_;
    out.evidence = evidence_;
    out.left_route = left_route_;
    out.right_route = right_route_;
    out.locked_route = locked_route_;
    out.blocker = blocker_;
    out.local_blocker = local_evidence_;
    out.blocker_association = blocker_.association;
    out.blocker_passed_latched = blocker_passed_latched_;
    out.start_clearance_recovery_used =
        locked_route_.start_clearance_recovery_used;
    out.entry_vehicle_progress = entry_vehicle_progress_;
    out.entry_blocker_progress = entry_blocker_route_progress_;
    out.entry_projection_dist = entry_vehicle_projection_dist_;
    out.entry_segment_index = entry_last_segment_index_;
    out.entry_progress_delta = entry_progress_delta_;
    out.entry_progress_max_delta = entry_progress_max_delta_;
    out.macro_route_progress = last_macro_route_progress_;
    out.macro_guide_lookahead = last_macro_guide_lookahead_;
    out.macro_guide_update_reason = last_macro_guide_update_reason_;
    out.macro_no_progress_duration = macro_no_progress_duration_;
    out.consecutive_failures_30hz = consecutive_failures_;
    out.macro_stable_exit_count = macro_stable_exit_count_;
    out.unknown_recovery_ticks = unknown_recovery_ticks_;
    out.unknown_recovery_active =
        static_cast<int>(unknown_recovery_ticks_) >=
        p_.macro_unknown_recovery_threshold_ticks;
    out.unknown_recovery_episode_count = unknown_recovery_episode_count_;
    out.macro_tick_event = macro_tick_event_;

    audit_.local_target_update_event = out.local_target.update_event;
    if (out.local.immediate_avoidance) ++audit_.immediate_avoidance_event;
    if (out.local.emergency_brake) ++audit_.emergency_brake_event;
    out.audit = audit_;
    out.transition = last_transition_;
    return out;
}

}  // namespace il_2d_multiscale_debug
