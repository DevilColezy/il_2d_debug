#include "il_2d_multiscale_debug/debug_simulation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace il_2d_multiscale_debug {

namespace {
// Fixed stream tag for the FIRST task of a new scene.  The first task's
// random seed depends ONLY on the scene seed + this tag — never on the
// historical task_counter_ — so the same scene seed always reproduces the
// same obstacles / start / goal / initial yaw regardless of how many tasks
// were generated before.
constexpr uint64_t kSceneFirstTaskTag = 0x5EEDF157ULL;

uint64_t deriveTaskSeed(uint64_t scene_seed, uint64_t task_counter) {
    uint64_t x = scene_seed ^ (task_counter * 0x9E3779B97F4A7C15ULL);
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31);
}
}  // namespace

DebugSimulation::DebugSimulation(const Params2D& p)
    : p_(p),
      scene_gen_(p),
      task_sampler_(p),
      raycaster_(p),
      vehicle_(p),
      fsm_(p) {}

bool DebugSimulation::newScene(uint64_t seed, std::string& reason) {
    // ── TRANSACTIONAL: build everything in temporary state first.  The
    // scene counter is only incremented after a FULLY successful commit.
    const uint64_t candidate_scene_id = scene_counter_;
    SceneGenerationResult sr = scene_gen_.generate(seed, candidate_scene_id);
    if (!sr.success) {
        // Keep the previous valid state (if any).  initialized_ is NOT
        // forced to false here: if a scene was already live it stays live;
        // on first startup it was already false.
        reason = sr.reason;
        return false;  // counter NOT incremented; previous state preserved
    }
    const Scene2D new_scene = sr.scene;

    TruthEsdf2D new_esdf;
    new_esdf.build(new_scene, p_.esdf_resolution);

    ConnectivityAnalyzer new_conn;
    new_conn.analyze(new_esdf, p_.scene_safety_clearance, p_.conn_neighbor);
    const double area_m2 =
        static_cast<double>(new_conn.mainComponentAreaCells()) *
        p_.esdf_resolution * p_.esdf_resolution;
    if (area_m2 < p_.conn_min_main_component_area_m2) {
        reason = "main component too small (" + std::to_string(area_m2) + " m^2)";
        return false;  // previous state preserved
    }

    Task2D new_task;
    std::string task_reason;
    // First task of a scene: seed from scene seed + FIXED stream tag only
    // (never from the historical task_counter_).
    if (!sampleTaskOn(new_scene, new_esdf, new_conn,
                      deriveTaskSeed(seed, kSceneFirstTaskTag), new_task,
                      task_reason)) {
        reason = task_reason;
        return false;  // previous state preserved
    }

    // ── Commit: swap in the new state only now.  commitTask is atomic
    //    (never fails once we are here), so the members are not left in a
    //    half-swapped state. ──
    scene_ = new_scene;
    esdf_ = new_esdf;
    conn_ = new_conn;
    obs_grid_.configure(scene_.min_bounds, scene_.max_bounds, p_.obs_resolution,
                        p_.obs_history_max_age_ticks);
    pending_goal_set_ = false;
    commitTask(new_task);
    ++scene_counter_;  // only after full success
    return true;
}

bool DebugSimulation::sampleTaskOn(const Scene2D& sc, const TruthEsdf2D& es,
                                   const ConnectivityAnalyzer& cn,
                                   uint64_t task_seed, Task2D& out,
                                   std::string& reason) {
    if (!task_sampler_.sampleTask(sc, es, cn, task_seed, task_counter_, out,
                                  reason)) {
        return false;
    }
    out.scene_id = sc.scene_id;
    return true;
}

void DebugSimulation::commitTask(const Task2D& task) {
    task_ = task;
    task_.task_id = task_counter_++;  // task id assigned on commit only
    initialized_ = true;              // a valid scene+task is now live
    // Capture the ORIGINAL sampled goal BEFORE any click can overwrite
    // task_.goal (used for CSV metadata: original_sampled_goal).
    original_sampled_goal_ = task_.goal;
    reinitEpisode();
}

void DebugSimulation::reinitEpisode() {
    // Reset all episode state.  tick_ = 0 FIRST, then the FSM is reset so
    // the INITIAL snapshot is consistent (tick 0, DIRECT_LOCAL, goal as
    // the current local target with effective event 0).  task_ is the
    // authoritative task (new after commit, identical after resetTask).
    tick_ = 0;
    processing_tick_ = 0;
    vehicle_.reset(VehicleState2D{task_.start, task_.initial_yaw, Vec2d::Zero(), 0.0});
    obs_grid_.reset();
    fsm_.reset(task_, tick_);
    executed_path_.clear();
    executed_path_.push_back(task_.start);
    current_truth_clearance_ = esdf_.sdfAt(task_.start);
    current_observed_clearance_ = std::numeric_limits<double>::infinity();
    truth_min_clearance_ = current_truth_clearance_;
    min_observed_clearance_ = std::numeric_limits<double>::infinity();
    pending_goal_set_ = false;
    last_patch_ = LocalObservation{};
    // Goal-semantics bookkeeping for the current episode: the goal live at
    // episode start, and the first acceptance flag (a pending goal accepted
    // at the first 5 Hz boundary becomes initial_accepted_goal).
    initial_accepted_goal_ = task_.goal;
    pending_goal_first_accept_ = true;

    // Build a valid INITIAL last_output_ + snapshot so the GUI shows the
    // correct state / goal / audit fields without a single step.
    last_output_ = FsmStepOutput{};
    last_output_.state = FsmState::DIRECT_LOCAL;
    last_output_.prev_state = FsmState::DIRECT_LOCAL;
    // buildSnapshot() reads prev_fsm_state / reason / tick from
    // last_output_.transition — the default TransitionRecord (prev =
    // DIRECT_LOCAL, reason = "", tick = 0) is exactly right for a fresh
    // episode, keeping the triplet internally consistent.
    last_output_.transition = TransitionRecord{};
    last_output_.local_target = LocalTarget{
        task_.goal, true, false, fsm_.effectiveLocalTargetEvent()};
    last_output_.macro_active = false;
    last_output_.audit.local_target_update_event =
        last_output_.local_target.update_event;
    buildSnapshot();
    flight_log_.clear();
    recordFlightLogSample();
}

bool DebugSimulation::newTaskInSameScene(uint64_t seed, bool use_default_seed,
                                         std::string& reason) {
    if (!initialized_) {
        reason = "simulation not initialized";
        return false;
    }
    Task2D task;
    const uint64_t tseed = use_default_seed
                               ? deriveTaskSeed(scene_.seed, task_counter_)
                               : seed;  // seed 0 is a legal explicit seed
    if (!sampleTaskOn(scene_, esdf_, conn_, tseed, task, reason)) return false;
    commitTask(task);  // assigns a new task_id (counter++) and re-inits
    reason = "ok; new task sampled";
    return true;
}

bool DebugSimulation::resetTask(std::string& reason) {
    if (!initialized_) {
        reason = "simulation not initialized";
        return false;
    }
    // Restart the CURRENT, IDENTICAL task: same scene / start / goal /
    // initial_yaw / task_id.  Re-initializes tick, vehicle state, local
    // observation history, executed path and FSM.  task_counter_ is NOT
    // incremented (unlike New Task).
    reinitEpisode();
    reason = "ok; current task restarted";
    return true;
}

bool DebugSimulation::setNavigationGoal(const Vec2d& goal, std::string& reason) {
    if (!initialized_) {
        reason = "simulation not initialized";
        return false;
    }
    if (terminal()) {
        // A terminal episode can never process a pending goal — reject it
        // explicitly instead of queueing one forever.
        reason = "episode terminal; reset or generate a new task";
        return false;
    }
    Vec2d snapped;
    if (!task_sampler_.snapToSelectable(scene_, esdf_, conn_, goal, snapped)) {
        reason = "no selectable cell near the requested point";
        return false;
    }
    // Store as PENDING — formally accepted by the upper layer at the next
    // 5 Hz boundary (acceptPendingGoal → fsm_.acceptNewGoal).  Vehicle
    // dynamics / local map / planner history are never reset.  Every
    // successful call increments the pending revision, so the GUI can
    // detect a CHANGED pending goal even when one was already pending.
    pending_goal_ = snapped;
    pending_goal_set_ = true;
    ++pending_goal_revision_;
    // Make the pending goal immediately observable to the GUI and logger;
    // formal acceptance still happens only at the next 5 Hz boundary.
    buildSnapshot();
    recordFlightLogSample();
    reason = "goal pending; will be accepted at next 5Hz boundary";
    return true;
}

bool DebugSimulation::terminal() const {
    const FsmState s = fsm_.state();
    return s == FsmState::GOAL_REACHED || s == FsmState::COLLISION ||
           s == FsmState::TIMEOUT || s == FsmState::TASK_INVALID;
}

FsmState DebugSimulation::state() const { return fsm_.state(); }

bool DebugSimulation::step() {
    if (!initialized_) return false;
    if (paused_) return false;
    if (terminal()) return false;
    return advance();
}

bool DebugSimulation::stepToNext5Hz() {
    if (!initialized_) return false;
    if (paused_) return false;
    if (terminal()) return false;
    // Record the macro tick counter BEFORE stepping; at least one 30 Hz
    // tick always runs, then we keep stepping until the counter strictly
    // increases — at that point the 5 Hz boundary decision has completed.
    // (Do NOT stop merely at a tick divisible by 6: that would park the
    // sim BEFORE the boundary decision executes.)
    const uint64_t start_event = fsm_.macroTickEvent();
    int guard = 0;
    bool advanced = false;
    while (guard++ < 60 && !terminal()) {
        if (!advance()) break;
        advanced = true;
        if (fsm_.macroTickEvent() > start_event) break;
    }
    return advanced && fsm_.macroTickEvent() > start_event;
}

bool DebugSimulation::advance() {
    processing_tick_ = tick_;

    // Only a goal already pending before the very first tick may redefine
    // `initial_accepted_goal`.  If tick 0 begins without one, the sampled
    // goal is final for this metadata field; later clicks are runtime goal
    // updates and must not rewrite episode-start semantics.
    if (tick_ == 0 && !pending_goal_set_) {
        pending_goal_first_accept_ = false;
    }

    // Formally accept a pending navigation goal exactly at a 5 Hz boundary:
    // the upper layer re-evaluates (keeping the locked side) and the 30 Hz
    // planner only ever sees the updated local target afterwards.
    if (pending_goal_set_ && tick_ % 6 == 0) {
        acceptPendingGoal();
    }

    // 1) Synthesize the instantaneous FOV observation and merge it into
    //    the short-term local map.
    last_patch_ = raycaster_.cast(vehicle_.state(), scene_, tick_);
    obs_grid_.integrate(last_patch_, tick_);

    // 2) FSM step (runs the 5 Hz macro expert when tick % 6 == 0).  The
    //    FSM receives BOTH the instantaneous FOV patch (fresh 5 Hz entry /
    //    side evidence) and the merged causal history map (5 Hz route
    //    continuity and the 30 Hz planner's view).
    FsmInput in{task_, vehicle_.state(), last_patch_, obs_grid_.observation(),
                tick_, /*collision=*/false};
    last_output_ = fsm_.step(in);

    // 3) Apply the command to the fixed-step simulator.
    const SimStepResult sim =
        vehicle_.step(last_output_.local.vx_body, last_output_.local.vy_body,
                      last_output_.local.yaw_rate, scene_);

    // 4) Track truth clearances (truth is only used by the referee /
    //    human display / macro expert — never the 30 Hz planner).
    current_truth_clearance_ = esdf_.sdfAt(vehicle_.state().position);
    truth_min_clearance_ =
        std::min(truth_min_clearance_, current_truth_clearance_);
    current_observed_clearance_ =
        obs_grid_.minClearanceToOccupied(vehicle_.state().position, 2.0);
    min_observed_clearance_ =
        std::min(min_observed_clearance_, current_observed_clearance_);

    // 5) Collision → freeze the episode (sim already zeroed velocity).
    if (sim.collision) {
        fsm_.forceCollision(last_output_);
    }

    executed_path_.push_back(vehicle_.state().position);
    ++tick_;

    buildSnapshot();
    recordFlightLogSample();
    return true;
}

void DebugSimulation::acceptPendingGoal() {
    task_.goal = pending_goal_;
    // The FIRST acceptance of the episode defines initial_accepted_goal
    // (when a pending goal was already live at tick 0).
    if (pending_goal_first_accept_) {
        initial_accepted_goal_ = pending_goal_;
        pending_goal_first_accept_ = false;
    }
    fsm_.acceptNewGoal(pending_goal_, tick_);
    pending_goal_set_ = false;
}

void DebugSimulation::buildSnapshot() {
    SimSnapshot& s = snapshot_;
    s.scene_id = scene_.scene_id;
    s.task_id = task_.task_id;
    s.seed = scene_.seed;
    s.tick = tick_;                    // completed 30 Hz steps
    s.processing_tick = processing_tick_;  // tick whose decision was computed
    s.time = time();                   // consistent with completed steps
    s.simulation_initialized = initialized_;
    s.paused = paused_;                // authoritative pause state
    s.fsm_state = last_output_.state;
    // previous state / transition reason / transition tick ALL come from
    // the SAME TransitionRecord (last_transition_) so the triplet is
    // internally consistent even when no transition happened this tick
    // (transition.prev holds the last real transition's previous state).
    s.prev_fsm_state = last_output_.transition.prev;
    s.transition_reason = last_output_.transition.reason;
    s.transition_tick = last_output_.transition.tick;
    s.macro_active = last_output_.macro_active;
    s.side = last_output_.side;
    s.side_reason = last_output_.evidence.reason;
    s.side_left_score = last_output_.evidence.left_score;
    s.side_right_score = last_output_.evidence.right_score;
    s.planner_success = last_output_.local.success;
    s.planner_failure_reason =
        failureReasonName(last_output_.local.failure_reason);
    s.target_bearing_error_deg = last_output_.local.target_bearing_error_deg;
    s.selected_terminal_heading_error_deg =
        last_output_.local.selected_terminal_heading_error_deg;
    s.selected_velocity_alignment_error_deg =
        last_output_.local.selected_velocity_alignment_error_deg;
    s.selected_cross_track_error_m =
        last_output_.local.selected_cross_track_error_m;
    s.selected_cost_total = last_output_.local.selected_cost_total;
    s.selected_cost_progress = last_output_.local.selected_cost_progress;
    s.selected_cost_clearance = last_output_.local.selected_cost_clearance;
    s.selected_cost_smoothness = last_output_.local.selected_cost_smoothness;
    s.selected_cost_speed_change = last_output_.local.selected_cost_speed_change;
    s.selected_cost_yaw_rate_change =
        last_output_.local.selected_cost_yaw_rate_change;
    s.selected_cost_terminal_heading =
        last_output_.local.selected_cost_terminal_heading;
    s.selected_cost_velocity_alignment =
        last_output_.local.selected_cost_velocity_alignment;
    s.selected_cost_cross_track = last_output_.local.selected_cost_cross_track;
    s.consecutive_failures_30hz = last_output_.consecutive_failures_30hz;
    s.macro_stable_exit_count = last_output_.macro_stable_exit_count;
    s.reject_not_known_free = last_output_.local.reject_not_known_free;
    s.reject_outside_current_fov = last_output_.local.reject_outside_current_fov;
    s.reject_observed_clearance_too_small =
        last_output_.local.reject_observed_clearance_too_small;
    s.reject_no_progress = last_output_.local.reject_no_progress;
    s.reject_other = last_output_.local.reject_other;
    s.reject_insufficient_braking_clearance =
        last_output_.local.reject_insufficient_braking_clearance;
    s.rejected_candidate_count = last_output_.local.rejectedCount();
    // Soft-clearance / dynamic-envelope diagnostics (v4).  NaN propagates
    // unchanged when no candidate was selected (explicit no-measurement).
    s.selected_soft_min_clearance_m =
        last_output_.local.selected_soft_min_clearance_m;
    s.selected_dynamic_required_clearance_m =
        last_output_.local.selected_dynamic_required_clearance_m;
    s.selected_closing_speed_mps =
        last_output_.local.selected_closing_speed_mps;
    s.handoff_clearance_m = last_output_.local.handoff_clearance_m;
    s.dynamic_clearance_blocked =
        last_output_.local.dynamic_clearance_blocked;
    s.local_limit_cycle_detected =
        last_output_.local.local_limit_cycle_detected;
    s.dynamic_window_candidate_count =
        last_output_.local.dynamic_window_candidate_count;
    // Intent / output / progress / status diagnostics (v5).
    s.selected_intent_vx_body = last_output_.local.intent_vx_body;
    s.selected_intent_vy_body = last_output_.local.intent_vy_body;
    s.selected_intent_yaw_rate = last_output_.local.intent_yaw_rate;
    s.selected_output_vx_body = last_output_.local.vx_body;
    s.selected_output_vy_body = last_output_.local.vy_body;
    s.selected_output_yaw_rate = last_output_.local.yaw_rate;
    s.selected_output_speed_mps = last_output_.local.selected_output_speed_mps;
    s.nominal_progress_m = last_output_.local.nominal_progress_m;
    s.executable_progress_m = last_output_.local.executable_progress_m;
    s.safe_prefix_duration_s = last_output_.local.safe_prefix_duration_s;
    s.candidate_progress_qualified =
        last_output_.local.candidate_progress_qualified;
    s.output_progress_qualified =
        last_output_.local.output_progress_qualified;
    s.progress_qualified = last_output_.local.progress_qualified;
    s.planner_status = last_output_.local.planner_status;
    s.planner_status_name = plannerStatusName(last_output_.local.planner_status);
    s.stationary_candidate_selected =
        last_output_.local.stationary_candidate_selected;
    s.stationary_selection_reason =
        last_output_.local.stationary_selection_reason;
    s.target_discontinuity_reset =
        last_output_.local.target_discontinuity_reset;
    s.target_mission_revision = last_output_.local.target_mission_revision;
    s.selected_cost_obstacle_risk =
        last_output_.local.selected_cost_obstacle_risk;
    // Early-avoidance risk diagnostics (v7).
    s.local_corridor_blocked = last_output_.local.local_corridor_blocked;
    s.first_blocking_obstacle_distance =
        last_output_.local.first_blocking_obstacle_distance;
    s.predicted_closest_clearance =
        last_output_.local.predicted_closest_clearance;
    s.time_to_collision = last_output_.local.time_to_collision;
    s.obstacle_risk_cost = last_output_.local.obstacle_risk_cost;
    s.avoidance_strength = last_output_.local.avoidance_strength;
    s.avoidance_active = last_output_.local.avoidance_active;
    s.local_target_distance = last_output_.local.local_target_distance;
    // Rolling macro guide diagnostics (v7).
    s.macro_route_progress = last_output_.macro_route_progress;
    s.macro_guide_lookahead = last_output_.macro_guide_lookahead;
    s.macro_guide_update_reason = last_output_.macro_guide_update_reason;
    s.macro_no_progress_duration = last_output_.macro_no_progress_duration;
    // Local-causal macro diagnostics (v8).
    s.macro_used_local_history_only =
        last_output_.macro_used_local_history_only;
    s.macro_guide_inside_current_fov =
        last_output_.macro_guide_inside_current_fov;
    s.macro_guide_endpoint_known_free =
        last_output_.macro_guide_endpoint_known_free;
    s.macro_guide_chord_known_free = last_output_.macro_guide_chord_known_free;
    s.macro_guide_min_observed_clearance =
        last_output_.macro_guide_min_observed_clearance;
    s.local_blocker_track_valid = last_output_.local_blocker_track_valid;
    s.local_blocker_behind = last_output_.local_blocker_behind;
    s.local_goal_corridor_clear = last_output_.local_goal_corridor_clear;
    s.local_leave_progress_m = last_output_.local_leave_progress_m;
    s.local_macro_route_valid = last_output_.local_macro_route_valid;
    s.relative_target_x_body = last_output_.relative_target_x_body;
    s.relative_target_y_body = last_output_.relative_target_y_body;
    s.target_bearing_deg = last_output_.target_bearing_deg;
    s.target_distance_m = last_output_.target_distance_m;
    // ── v9: 5 Hz visibility target corrector + target encoding ─────
    s.target_correction_type = last_output_.target_correction_type;
    s.target_correction_type_name =
        last_output_.target_correction_type_name;
    s.target_correction_active = last_output_.target_correction_active;
    s.target_direction_token = last_output_.target_direction_token;
    s.target_direction_x_body = last_output_.target_direction_x_body;
    s.target_direction_y_body = last_output_.target_direction_y_body;
    s.target_distance_normalized = last_output_.target_distance_normalized;
    s.effective_target_x = last_output_.effective_target_x;
    s.effective_target_y = last_output_.effective_target_y;
    s.effective_target_world_valid =
        last_output_.effective_target_world_valid;
    s.original_goal = last_output_.original_goal;
    s.observability_goal_inside_fov =
        last_output_.observability_goal_inside_fov;
    s.observability_direct_corridor_blocked =
        last_output_.observability_direct_corridor_blocked;
    s.observability_blocker_observed =
        last_output_.observability_blocker_observed;
    s.observability_left_bypass_visible =
        last_output_.observability_left_bypass_visible;
    s.observability_right_bypass_visible =
        last_output_.observability_right_bypass_visible;
    s.observability_local_avoidance_observable =
        last_output_.observability_local_avoidance_observable;
    s.observability_fov_boundary_truncated =
        last_output_.observability_fov_boundary_truncated;
    s.observability_unknown_occluded =
        last_output_.observability_unknown_occluded;
    s.observability_reason = last_output_.observability_reason;
    s.observability_left_score = last_output_.observability_left_score;
    s.observability_right_score = last_output_.observability_right_score;
    s.correction_enter_event = last_output_.correction_enter_event;
    s.correction_exit_event = last_output_.correction_exit_event;
    s.correction_update_event = last_output_.correction_update_event;
    s.start_clearance_recovery_used =
        last_output_.start_clearance_recovery_used;
    s.unknown_recovery_ticks = last_output_.unknown_recovery_ticks;
    s.unknown_recovery_active = last_output_.unknown_recovery_active;
    s.unknown_recovery_episode_count = last_output_.unknown_recovery_episode_count;
    s.local_target_update_event = last_output_.audit.local_target_update_event;
    s.macro_tick_event = last_output_.macro_tick_event;
    // v8: the "blocker id" is the LOCAL blocker track id (never a truth
    // obstacle id — the privileged association is gone).
    s.blocker_id = last_output_.local_blocker_track_id;
    s.blocker_association = last_output_.blocker_association;
    s.blocker_passed_latched = last_output_.blocker_passed_latched;
    s.entry_vehicle_progress = last_output_.entry_vehicle_progress;
    s.entry_blocker_progress = last_output_.entry_blocker_progress;
    s.entry_projection_dist = last_output_.entry_projection_dist;
    s.entry_segment_index = last_output_.entry_segment_index;
    s.entry_progress_delta = last_output_.entry_progress_delta;
    s.entry_progress_max_delta = last_output_.entry_progress_max_delta;
    s.accepted_goal_event = fsm_.acceptedGoalEvent();
    s.reentry_guard_ticks = fsm_.reentryGuardTicks();
    s.local_target_updated_this_tick = last_output_.local_target_updated;
    s.macro_tick_ran_this_tick = last_output_.macro_tick_ran;
    s.pending_goal_set = pending_goal_set_;
    s.pending_goal_revision = pending_goal_revision_;
    s.pending_goal_position = pending_goal_set_ ? pending_goal_ : Vec2d(0.0, 0.0);
    s.current_observed_clearance = current_observed_clearance_;
    s.current_truth_clearance = current_truth_clearance_;
    s.min_observed_clearance = min_observed_clearance_;
    s.truth_min_clearance = truth_min_clearance_;
    s.goal_reached = last_output_.state == FsmState::GOAL_REACHED;
    s.collision = last_output_.state == FsmState::COLLISION;
    s.task_invalid = last_output_.state == FsmState::TASK_INVALID;
    s.audit = last_output_.audit;
    // First-obstacle-observed audit event comes from the observed grid.
    s.audit.obstacle_first_observed_event =
        obs_grid_.obstacleFirstObservedEvent();
    s.vehicle_position = vehicle_.state().position;
    s.yaw = vehicle_.state().yaw;
    s.local_target = last_output_.local_target.position;
    s.local_target_valid = last_output_.local_target.valid;
    s.local_target_is_macro_guide = last_output_.local_target.is_macro_guide;
}

void DebugSimulation::recordFlightLogSample() {
    if (!initialized_) return;

    FlightLogSample row;
    row.snapshot = snapshot_;
    const VehicleState2D& vehicle_state = vehicle_.state();
    row.velocity_world = vehicle_state.velocity_world;
    row.vehicle_yaw_rate = vehicle_state.yaw_rate;
    row.final_goal = task_.goal;
    row.cmd_vx_body = last_output_.local.vx_body;
    row.cmd_vy_body = last_output_.local.vy_body;
    row.cmd_yaw_rate = last_output_.local.yaw_rate;
    row.turn_mode = last_output_.local.turn_mode;
    row.blocked_observed = last_output_.local.blocked_observed;
    row.immediate_avoidance = last_output_.local.immediate_avoidance;
    row.emergency_brake = last_output_.local.emergency_brake;
    row.selected_trajectory_points = static_cast<uint32_t>(
        last_output_.local.selected.points.size());
    row.rejected_candidate_count = static_cast<uint32_t>(
        last_output_.local.rejected_candidates.size());

    // A GUI goal click can update pending-goal state without advancing
    // simulation time.  Replace that tick's row so the log describes the
    // exact value that will be consumed at the next 5 Hz boundary.
    if (!flight_log_.empty() &&
        flight_log_.back().snapshot.tick == row.snapshot.tick) {
        flight_log_.back() = row;
    } else {
        flight_log_.push_back(row);
    }
}

}  // namespace il_2d_multiscale_debug
