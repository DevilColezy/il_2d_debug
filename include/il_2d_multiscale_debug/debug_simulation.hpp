#pragma once
/// @file   debug_simulation.hpp
/// @brief  ROS-free deterministic simulation orchestrator.
///
/// Owns the scene, truth ESDF, connectivity, task, observation pipeline,
/// 30 Hz planner, 5 Hz macro expert, FSM and planar vehicle simulator.
/// Exposes a tick-accurate step() / stepToNext5Hz() API and every piece
/// of per-tick state needed by the node for publishing.  Pause is
/// enforced here: step() refuses to advance while paused, and the wall
/// timer in the node can therefore never sneak ticks forward.

#include "il_2d_multiscale_debug/connectivity_analyzer.hpp"
#include "il_2d_multiscale_debug/fov_raycaster_2d.hpp"
#include "il_2d_multiscale_debug/hierarchical_expert_fsm.hpp"
#include "il_2d_multiscale_debug/local_planner_30hz.hpp"
#include "il_2d_multiscale_debug/observed_grid_2d.hpp"
#include "il_2d_multiscale_debug/planar_vehicle_simulator.hpp"
#include "il_2d_multiscale_debug/scene_generator.hpp"
#include "il_2d_multiscale_debug/task_sampler.hpp"
#include "il_2d_multiscale_debug/truth_esdf_2d.hpp"
#include "il_2d_multiscale_debug/types.hpp"

#include <string>
#include <vector>

namespace il_2d_multiscale_debug {

/// Lightweight per-tick status snapshot (node → DebugSnapshot.msg).
struct SimSnapshot {
    uint64_t scene_id = 0, task_id = 0, seed = 0, tick = 0;
    uint64_t processing_tick = 0;  // tick whose decision this snapshot describes
    double time = 0.0;
    bool simulation_initialized = false;
    bool paused = true;  // authoritative pause state (node/launch driven)
    FsmState fsm_state = FsmState::DIRECT_LOCAL;
    FsmState prev_fsm_state = FsmState::DIRECT_LOCAL;
    std::string transition_reason;
    uint64_t transition_tick = 0;
    bool macro_active = false;
    SideSelection side = SideSelection::NONE;
    std::string side_reason;
    double side_left_score = 0.0;   // average visible free range (m)
    double side_right_score = 0.0;  // average visible free range (m)
    bool planner_success = false;
    std::string planner_failure_reason;
    uint32_t consecutive_failures_30hz = 0;
    uint32_t macro_stable_exit_count = 0;
    // Selected-candidate target-tracking diagnostics (deg/m / cost parts).
    double target_bearing_error_deg = 0.0;
    double selected_terminal_heading_error_deg = 0.0;
    double selected_velocity_alignment_error_deg = 0.0;
    double selected_cross_track_error_m = 0.0;
    double selected_cost_total = 0.0;
    double selected_cost_progress = 0.0;
    double selected_cost_clearance = 0.0;
    double selected_cost_smoothness = 0.0;
    double selected_cost_speed_change = 0.0;
    double selected_cost_yaw_rate_change = 0.0;
    double selected_cost_terminal_heading = 0.0;
    double selected_cost_velocity_alignment = 0.0;
    double selected_cost_cross_track = 0.0;
    // Per-tick candidate rejection diagnostics (sum == rejected count).
    uint32_t reject_not_known_free = 0;
    uint32_t reject_outside_current_fov = 0;
    uint32_t reject_observed_clearance_too_small = 0;
    uint32_t reject_no_progress = 0;
    uint32_t reject_other = 0;
    uint32_t reject_insufficient_braking_clearance = 0;
    uint32_t rejected_candidate_count = 0;  // == sum of the six counters
    // ── Soft-clearance / dynamic-envelope diagnostics (v4) ──────────
    // NaN when no candidate was selected (explicit "no measurement").
    double selected_soft_min_clearance_m = std::numeric_limits<double>::quiet_NaN();
    double selected_dynamic_required_clearance_m = std::numeric_limits<double>::quiet_NaN();
    double selected_closing_speed_mps = std::numeric_limits<double>::quiet_NaN();
    double handoff_clearance_m = std::numeric_limits<double>::quiet_NaN();
    bool dynamic_clearance_blocked = false;
    bool local_limit_cycle_detected = false;
    uint32_t dynamic_window_candidate_count = 0;
    // ── Intent / output / progress / status diagnostics (v5) ───────
    // The long-term rollout INTENT vs the EXECUTABLE OUTPUT actually sent
    // to the simulator (selected_output_* == cmd_* in the flight log).
    double selected_intent_vx_body = 0.0;
    double selected_intent_vy_body = 0.0;
    double selected_intent_yaw_rate = 0.0;
    double selected_output_vx_body = 0.0;
    double selected_output_vy_body = 0.0;
    double selected_output_yaw_rate = 0.0;
    // Magnitude of the EXECUTABLE OUTPUT command (hypot of output vx/vy).
    double selected_output_speed_mps = 0.0;
    double nominal_progress_m = 0.0;       // over the 2.5 s nominal rollout
    double executable_progress_m = 0.0;    // over the certified safe prefix
    double safe_prefix_duration_s = 0.0;   // certified prefix duration (s)
    bool candidate_progress_qualified = false;  // before supervisor override
    bool output_progress_qualified = false;     // final command progresses
    bool progress_qualified = false;       // compatibility alias of output
    PlannerStatus planner_status = PlannerStatus::NO_SAFE_CANDIDATE;
    std::string planner_status_name;       // derived from planner_status
    bool stationary_candidate_selected = false;
    std::string stationary_selection_reason;
    bool target_discontinuity_reset = false;
    uint64_t target_mission_revision = 0;
    double selected_cost_obstacle_risk = 0.0;
    // ── Continuous early-avoidance risk diagnostics (v7) ───────────
    // True when an observed obstacle intersects the local vehicle →
    // LocalTarget corridor ahead of the vehicle.
    bool local_corridor_blocked = false;
    // Longitudinal distance (m) to the first obstacle blocking the corridor
    // (NaN when clear).
    double first_blocking_obstacle_distance =
        std::numeric_limits<double>::quiet_NaN();
    // Selected candidate's predicted closest clearance (m) to any observed
    // obstacle (closest approach of the nominal rollout).
    double predicted_closest_clearance =
        std::numeric_limits<double>::quiet_NaN();
    // Selected candidate's time to the closest approach (s).
    double time_to_collision = std::numeric_limits<double>::quiet_NaN();
    // Selected candidate's normalized obstacle-risk cost term.
    double obstacle_risk_cost = 0.0;
    // Normalized avoidance strength (== obstacle_risk_cost).
    double avoidance_strength = 0.0;
    bool avoidance_active = false;
    // Distance (m) from the vehicle to the current LocalTarget.
    double local_target_distance = 0.0;
    // ── Rolling macro guide diagnostics (v7) ───────────────────────
    // Guide arc (m) along the current locked route.
    double macro_route_progress = -1.0;
    // Arc-length lookahead (m) used to select the guide.
    double macro_guide_lookahead = 0.0;
    // Why the guide was (re)selected this 5 Hz tick (v8: local reasons).
    std::string macro_guide_update_reason = "";
    // Continuous time (s) the vehicle has failed to translate in macro mode.
    double macro_no_progress_duration = 0.0;
    // ── Local-causal macro diagnostics (v8) ────────────────────────
    // 5 Hz expert ran using ONLY local observations (always true).
    bool macro_used_local_history_only = false;
    // Hard-certification results of the current macro guide.
    bool macro_guide_inside_current_fov = false;
    bool macro_guide_endpoint_known_free = false;
    bool macro_guide_chord_known_free = false;
    // Min observed clearance along the guide chord (m, inf when not
    // certified).
    double macro_guide_min_observed_clearance =
        std::numeric_limits<double>::infinity();
    // Local blocker track state (id also published as blocker_id).
    bool local_blocker_track_valid = false;
    bool local_blocker_behind = false;
    bool local_goal_corridor_clear = false;
    // Goal-distance reduction (m) since macro entry.
    double local_leave_progress_m = 0.0;
    bool local_macro_route_valid = false;
    // Body-frame supervision of the macro guide (training/logging).
    double relative_target_x_body = 0.0;
    double relative_target_y_body = 0.0;
    double target_bearing_deg = 0.0;
    double target_distance_m = 0.0;
    // ── v9: 5 Hz visibility target corrector + target encoding ─────
    // The directive type actually driving the 30 Hz expert this tick.
    uint8_t target_correction_type = 0;  // TargetCorrectionType
    std::string target_correction_type_name = "PASS_THROUGH";
    bool target_correction_active = false;
    // Student direction class (0=TURN_LEFT, 1..N ordinary, N+1=TURN_RIGHT,
    // -1 for PASS_THROUGH).
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
    // ── 5 Hz local observability diagnostics ───────────────────────
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
    // ── Correction events ──────────────────────────────────────────
    uint64_t correction_enter_event = 0;
    uint64_t correction_exit_event = 0;
    uint64_t correction_update_event = 0;
    // Macro A* start-clearance recovery was used on the locked route
    // (v8: always false — no global A*).
    bool start_clearance_recovery_used = false;
    // LOCAL_UNKNOWN_RECOVERY diagnostic (UNKNOWN-driven, never macro).
    uint32_t unknown_recovery_ticks = 0;
    bool unknown_recovery_active = false;
    uint32_t unknown_recovery_episode_count = 0;
    uint64_t local_target_update_event = 0;
    uint64_t macro_tick_event = 0;
    int blocker_id = -1;
    BlockerAssociation blocker_association = BlockerAssociation::NONE;
    bool blocker_passed_latched = false;
    double entry_vehicle_progress = -1.0;
    double entry_blocker_progress = -1.0;
    double entry_projection_dist = -1.0;
    int32_t entry_segment_index = -1;
    double entry_progress_delta = 0.0;
    double entry_progress_max_delta = 0.0;
    uint64_t accepted_goal_event = 0;
    int reentry_guard_ticks = 0;
    bool local_target_updated_this_tick = false;
    bool macro_tick_ran_this_tick = false;
    bool pending_goal_set = false;
    uint64_t pending_goal_revision = 0;
    Vec2d pending_goal_position{0.0, 0.0};
    // Instantaneous clearances at the current vehicle position.  The two
    // legacy min_* fields below remain episode-wide cumulative minima.
    double current_observed_clearance = std::numeric_limits<double>::infinity();
    double current_truth_clearance = std::numeric_limits<double>::infinity();
    double min_observed_clearance = std::numeric_limits<double>::infinity();
    double truth_min_clearance = std::numeric_limits<double>::infinity();
    bool goal_reached = false, collision = false, task_invalid = false;
    AuditFlags audit;
    Vec2d vehicle_position{0.0, 0.0};
    double yaw = 0.0;
    Vec2d local_target{0.0, 0.0};
    bool local_target_valid = false;
    bool local_target_is_macro_guide = false;
};

/// One lossless 30 Hz row used by the on-demand task log exporter.
/// The vector is reset whenever an episode is reset or a new task/scene
/// is committed.  It is deliberately kept in the ROS-free core so a
/// multi-tick GUI step cannot skip intermediate 30 Hz decisions.
struct FlightLogSample {
    SimSnapshot snapshot;
    Vec2d velocity_world{0.0, 0.0};
    double vehicle_yaw_rate = 0.0;
    Vec2d final_goal{0.0, 0.0};
    double cmd_vx_body = 0.0;
    double cmd_vy_body = 0.0;
    double cmd_yaw_rate = 0.0;
    bool turn_mode = false;
    bool blocked_observed = false;
    bool immediate_avoidance = false;
    bool emergency_brake = false;
    uint32_t selected_trajectory_points = 0;
    uint32_t rejected_candidate_count = 0;
};

class DebugSimulation {
public:
    explicit DebugSimulation(const Params2D& p);
    ~DebugSimulation() = default;

    // ── Lifecycle / controls ───────────────────────────────────────
    /// Transactional scene generation: builds the new scene / ESDF /
    /// connectivity / task in temporary state and only swaps them in on
    /// full success.  On failure the previous scene + task (and every
    /// counter / FSM / snapshot) are preserved — initialized_ is never
    /// forced to false just because a NEW scene failed.
    bool newScene(uint64_t seed, std::string& reason);
    /// Sample a NEW task inside the current scene (new start/goal/yaw).
    ///   use_default_seed=true  → derive from the scene seed;
    ///   use_default_seed=false → use `seed` verbatim (0 is legal) and the
    ///                            same scene+task seed reproduces the task.
    /// On success task_counter_ / task_id increase.
    bool newTaskInSameScene(uint64_t seed, bool use_default_seed, std::string& reason);
    /// Restart the CURRENT, IDENTICAL task: same scene, start, goal,
    /// initial_yaw and task_id.  Clears tick, vehicle state, local
    /// observation history, executed path and FSM.  task_counter_ is NOT
    /// incremented.  seed/use_default_seed are intentionally absent.
    bool resetTask(std::string& reason);
    /// Store a PENDING final navigation goal (snapped to selectable
    /// space).  Formally accepted by the upper layer at the next 5 Hz
    /// boundary.  Returns false only when no selectable cell is nearby.
    bool setNavigationGoal(const Vec2d& goal, std::string& reason);
    bool hasPendingGoal() const { return pending_goal_set_; }

    bool setPaused(bool p) {
        paused_ = p;
        snapshot_.paused = p;
        return true;
    }
    bool paused() const { return paused_; }

    /// True once a scene+task has been committed successfully.  While
    /// false, dependent services must refuse with "simulation not
    /// initialized".  GenerateScene may still commit another seed and
    /// re-initialize.
    bool initialized() const { return initialized_; }

    /// Advance exactly one 30 Hz tick.  Returns false when paused or
    /// terminal (nothing advanced).
    bool step();
    /// Advance until the next 5 Hz boundary completes.  Returns false if
    /// paused/terminal.
    bool stepToNext5Hz();

    // ── Accessors for publishing ───────────────────────────────────
    uint64_t tick() const { return tick_; }
    uint64_t processingTick() const { return processing_tick_; }
    double time() const { return tick_ / 30.0; }
    uint64_t sceneId() const { return scene_.scene_id; }
    uint64_t taskId() const { return task_.task_id; }
    uint64_t seed() const { return scene_.seed; }
    bool terminal() const;
    FsmState state() const;

    const Scene2D& scene() const { return scene_; }
    const TruthEsdf2D& esdf() const { return esdf_; }
    const ConnectivityAnalyzer& connectivity() const { return conn_; }
    const Task2D& task() const { return task_; }
    const ObservedGrid2D& observedGrid() const { return obs_grid_; }
    const PlanarVehicleSimulator& vehicle() const { return vehicle_; }
    const FsmStepOutput& lastOutput() const { return last_output_; }
    const SimSnapshot& snapshot() const { return snapshot_; }
    const std::vector<FlightLogSample>& flightLog() const { return flight_log_; }
    const std::vector<Vec2d>& executedPath() const { return executed_path_; }
    const LocalObservation& lastPatch() const { return last_patch_; }
    uint64_t effectiveLocalTargetEvent() const {
        return fsm_.effectiveLocalTargetEvent();
    }
    /// Monotonic final-goal revision counter (visualization can detect
    /// that the final navigation goal changed).
    uint64_t acceptedGoalEvent() const { return fsm_.acceptedGoalEvent(); }
    /// World position of the PENDING (not yet accepted) navigation goal.
    Vec2d pendingGoal() const { return pending_goal_; }
    /// Monotonic revision of the pending goal — increments on EVERY
    /// successful setNavigationGoal (even when a goal was already pending).
    uint64_t pendingGoalRevision() const { return pending_goal_revision_; }
    /// The goal sampled with the current task (never overwritten by clicks).
    Vec2d originalSampledGoal() const { return original_sampled_goal_; }
    /// The goal that was live after the episode's first 5 Hz acceptance (or
    /// the sampled goal if no pending goal was accepted before it).
    Vec2d initialAcceptedGoal() const { return initial_accepted_goal_; }

private:
    /// Sample a task for the given (scene, esdf, conn) into `out` without
    /// committing any member state.  task_id is assigned on commit only.
    bool sampleTaskOn(const Scene2D& sc, const TruthEsdf2D& es,
                      const ConnectivityAnalyzer& cn, uint64_t task_seed,
                      Task2D& out, std::string& reason);
    /// Commit a sampled task (atomic: never fails after the members have
    /// been swapped) and build a valid INITIAL snapshot immediately.
    void commitTask(const Task2D& task);
    /// Re-initialize the current episode from task_ (tick=0, vehicle,
    /// observation history, executed path, FSM, initial snapshot).  Used
    /// by commitTask (after a task change) and by resetTask (same task).
    void reinitEpisode();
    /// Accept a pending navigation goal at a 5 Hz boundary.
    void acceptPendingGoal();
    bool advance();  // one tick, no pause/terminal check
    void buildSnapshot();
    void recordFlightLogSample();

    Params2D p_;
    uint64_t tick_ = 0;             // completed 30 Hz steps
    uint64_t processing_tick_ = 0;  // tick being/just processed
    bool paused_ = true;
    bool initialized_ = false;  // true only after a scene+task commit

    SceneGenerator scene_gen_;
    TruthEsdf2D esdf_;
    ConnectivityAnalyzer conn_;
    TaskSampler task_sampler_;
    FovRaycaster2D raycaster_;
    ObservedGrid2D obs_grid_;
    PlanarVehicleSimulator vehicle_;
    HierarchicalExpertFsm fsm_;

    Scene2D scene_;
    Task2D task_;
    uint64_t scene_counter_ = 0;
    uint64_t task_counter_ = 0;

    // pending navigation goal (formally accepted at the next 5 Hz boundary)
    Vec2d pending_goal_{0.0, 0.0};
    bool pending_goal_set_ = false;
    uint64_t pending_goal_revision_ = 0;  // increments on every successful set
    // goal-semantics bookkeeping (CSV metadata)
    Vec2d original_sampled_goal_{0.0, 0.0};  // sampled task goal, immutable
    Vec2d initial_accepted_goal_{0.0, 0.0};  // goal live after first acceptance
    bool pending_goal_first_accept_ = true;

    LocalObservation last_patch_;
    FsmStepOutput last_output_;
    SimSnapshot snapshot_;
    std::vector<Vec2d> executed_path_;
    std::vector<FlightLogSample> flight_log_;

    double truth_min_clearance_ = std::numeric_limits<double>::infinity();
    double min_observed_clearance_ = std::numeric_limits<double>::infinity();
    double current_truth_clearance_ = std::numeric_limits<double>::infinity();
    double current_observed_clearance_ = std::numeric_limits<double>::infinity();
};

}  // namespace il_2d_multiscale_debug
