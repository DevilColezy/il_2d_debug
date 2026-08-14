# ===================================================================
#  DebugSnapshot.msg
#  Lightweight per-tick status message for the debug panel and for
#  manual information-leak / expert-timing auditing.
#
#  Fields marked [AUDIT] are the explicit audit fields required to be
#  displayed in the GUI.
#
#  TICK SEMANTICS:
#    tick           = number of COMPLETED 30 Hz steps (simulation time
#                     equals tick / 30.0 — consistent with the finished
#                     steps; "Reset current task" starts at tick = 0).
#    processing_tick= the 30 Hz tick whose decision this snapshot
#                     describes (== tick when idle before any step, and
#                     == tick-1 after a completed step).
# ===================================================================
std_msgs/Header header

# ── Identity ─────────────────────────────────────────────────────
uint64 scene_id
uint64 task_id
uint64 seed
uint64 tick                 # completed 30 Hz steps (0-based)
uint64 processing_tick      # tick currently being processed (see above)
float64 time                # tick / 30.0
bool   simulation_initialized   # false until a scene+task committed
bool   paused               # authoritative pause state published by the node

# ── FSM ──────────────────────────────────────────────────────────
uint8  fsm_state            # see FsmState enum in types.hpp
string fsm_state_name
# previous_fsm_state / transition_tick / transition_reason are read from
# the SAME TransitionRecord (the last real transition), so the triplet is
# always internally consistent even between transitions.
uint8  previous_fsm_state
string previous_fsm_state_name
uint64 transition_tick      # tick at which the last transition happened
string transition_reason    # current/last transition reason
bool   macro_active
uint8  side                 # 0 NONE, 1 LEFT, 2 RIGHT
string side_reason          # VISIBLE_LEFT_BETTER / VISIBLE_RIGHT_BETTER / AMBIGUOUS_DEFAULT_RIGHT

# ── Blocker / macro progress (audit) ─────────────────────────────
uint8  blocker_association  # 0 NONE, 1 MATCHED, 2 NO_MATCH, 3 AMBIGUOUS_MATCH
string blocker_association_name
bool   blocker_passed_latched   # latched once the fixed-route progress passes
bool   start_clearance_recovery_used  # macro A* start recovery prefix used
float64 entry_vehicle_progress  # monotonic arc length along fixed entry route
float64 entry_blocker_progress  # fixed blocker arc length on the entry route
float64 entry_projection_dist   # last lateral projection distance to the route
int32   entry_segment_index     # last matched segment on the fixed entry route
float64 entry_progress_delta    # accepted progress increase on this macro tick
float64 entry_progress_max_delta # configured physical upper bound per macro tick

# ── 30 Hz planner ─────────────────────────────────────────────────
bool   planner_success
string planner_failure_reason
uint32 consecutive_failures_30hz
float64 min_observed_clearance
float64 truth_min_clearance
float64 current_observed_clearance  # current pose, not episode minimum
float64 current_truth_clearance     # current pose, not episode minimum

# ── Soft-clearance / dynamic-envelope diagnostics (v4) ─────────────
# selected_* fields are NaN when no candidate was selected (explicit
# "no measurement" — never 0, so a real 0 reading is unambiguous).
float64 selected_soft_min_clearance_m        # actual measured clearance (m)
float64 selected_dynamic_required_clearance_m  # max dynamic required (m)
float64 selected_closing_speed_mps           # max closing speed (m/s)
float64 handoff_clearance_m                  # static macro-handoff base (m)
bool   dynamic_clearance_blocked             # dynamic envelope blocked plan
bool   local_limit_cycle_detected            # stall/oscillation detector fired
uint32 dynamic_window_candidate_count        # candidates after 30Hz window dedup

# ── Candidate rejection diagnostics (per-tick, first reason each) ──
uint32 reject_not_known_free          # rejected: sample not known-FREE
uint32 reject_outside_current_fov     # rejected: safe prefix left current FOV
uint32 reject_observed_clearance_too_small  # rejected: hard observed clearance
uint32 reject_no_progress             # rejected: no progress on safe prefix
uint32 reject_other                   # rejected: other / no usable prefix
uint32 reject_insufficient_braking_clearance  # rejected: dynamic envelope
uint32 rejected_candidate_count       # total rejected candidates this tick
                                       # (== sum of the six counters)

# ── Selected-candidate target-tracking diagnostics ────────────────
# Filled for the best feasible candidate every tick (0 when turn mode or
# no candidate was selected).  Reference line = current vehicle → current
# LocalTarget (NOT a global path; rebuilt every tick from the local target).
float64 target_bearing_error_deg             # |bearing| of LocalTarget
float64 selected_terminal_heading_error_deg  # heading at prefix end vs target dir
float64 selected_velocity_alignment_error_deg # velocity dir vs target dir
float64 selected_cross_track_error_m         # mean lateral offset (local ref line)
float64 selected_cost_total
float64 selected_cost_progress
float64 selected_cost_clearance
float64 selected_cost_smoothness
float64 selected_cost_speed_change
float64 selected_cost_yaw_rate_change
float64 selected_cost_terminal_heading
float64 selected_cost_velocity_alignment
float64 selected_cost_cross_track

# ── Local UNKNOWN recovery diagnostic ─────────────────────────────
# NO_SAFE_CANDIDATE (UNKNOWN-driven) never triggers the 5 Hz macro expert
# and is never mislabelled as real observed blockage.  These fields expose
# how long the planner has been stuck without a safe candidate.
uint32 unknown_recovery_ticks         # consecutive NO_SAFE_CANDIDATE ticks
bool   unknown_recovery_active        # >= macro/unknown_recovery_threshold_ticks
uint32 unknown_recovery_episode_count # monotonic recovery episodes

# ── Events / counts ───────────────────────────────────────────────
uint64 local_target_update_event   # [AUDIT] monotonic effective event
uint64 macro_tick_event
uint32 macro_stable_exit_count
int32  blocker_id
uint64 accepted_goal_event         # monotonic final-goal revision counter
int32  macro_reentry_guard_ticks   # remaining guard (30 Hz ticks)
bool   local_target_updated_this_tick
bool   macro_tick_ran_this_tick

# ── Pending navigation goal ───────────────────────────────────────
bool    pending_goal_set          # a goal is queued for the next 5 Hz boundary
uint64  pending_goal_revision     # increments on EVERY successful setNavigationGoal
float64[2] pending_goal_position  # world position of the pending goal (0,0 if none)

# ── Terminal flags ────────────────────────────────────────────────
bool   goal_reached
bool   collision
bool   task_invalid

# ── Audit fields ──────────────────────────────────────────────────
bool used_truth_by_local_planner     # [AUDIT] always false by construction
bool used_global_esdf_by_local_planner  # [AUDIT] always false by construction
bool used_global_path_by_local_planner  # [AUDIT] always false by construction
bool macro_used_privileged_esdf      # [AUDIT]
bool side_selected_from_visible_evidence  # [AUDIT]
bool side_ambiguous_defaulted_right  # [AUDIT]
bool side_selected_using_current_patch   # [AUDIT] side evidence used current_patch
float64 side_left_score              # [AUDIT] average visible free range, LEFT
float64 side_right_score             # [AUDIT] average visible free range, RIGHT
uint64 macro_enter_event             # [AUDIT]
uint64 macro_exit_event              # [AUDIT]
uint64 obstacle_first_observed_event # [AUDIT]
uint64 immediate_avoidance_event     # [AUDIT]
uint64 emergency_brake_event         # [AUDIT]

# ── Vehicle / target ──────────────────────────────────────────────
float64[2] vehicle_position
float64    yaw
float64[2] local_target
bool       local_target_valid
bool       local_target_is_macro_guide

# ── Intent / output / progress / status (v5) ──────────────────────
# selected_intent_* = the long-term ROLLOUT INTENT (what the planner wants
# to do over 2.5 s); selected_output_* = the EXECUTABLE OUTPUT (the one
# control-period reachable command actually sent to the simulator).
# selected_output_vx/vy/yaw_rate equal cmd_vx_body/cmd_vy_body/cmd_yaw_rate
# in the flight-log CSV.
float64 selected_intent_vx_body     # intent body-forward velocity (m/s)
float64 selected_intent_vy_body     # intent body-lateral velocity (m/s)
float64 selected_intent_yaw_rate    # intent yaw rate (rad/s)
float64 selected_output_vx_body     # executable body-forward velocity (m/s)
float64 selected_output_vy_body     # executable body-lateral velocity (m/s)
float64 selected_output_yaw_rate    # executable yaw rate (rad/s)
float64 selected_output_speed_mps   # |executable output| (m/s)
float64 nominal_progress_m          # progress over the 2.5 s nominal rollout (m)
float64 executable_progress_m       # progress over the certified safe prefix (m)
float64 safe_prefix_duration_s      # certified executable prefix duration (s)
bool   candidate_progress_qualified # best safe candidate before supervision
bool   output_progress_qualified    # final command actually has progress
bool   progress_qualified           # compatibility alias of output progress
uint8  planner_status               # PlannerStatus enum (types.hpp)
string planner_status_name
bool   stationary_candidate_selected  # executable output is (near-)zero
string stationary_selection_reason    # terminal_settling / turn_mode / ...
bool   target_discontinuity_reset     # this tick: target jumped, memory reset
uint64 target_mission_revision        # mission revision (only this resets memory)
float64 selected_cost_obstacle_risk   # normalized early-avoidance risk term (v7)

# ── Continuous early-avoidance risk (v7) ──────────────────────────
bool   local_corridor_blocked         # observed obstacle intersects local corridor
float64 first_blocking_obstacle_distance  # dist to first corridor blocker (m, NaN if clear)
float64 predicted_closest_clearance   # selected candidate closest approach (m, +inf if none)
float64 time_to_collision             # first entry into nominal-clearance shell (s, +inf if none)
float64 obstacle_risk_cost            # selected candidate normalized risk term
float64 avoidance_strength            # normalized avoidance strength (== risk)
bool   avoidance_active               # avoidance_strength >= threshold
float64 local_target_distance         # distance to current LocalTarget (m)

# ── Rolling macro guide diagnostics (v7) ──────────────────────────
float64 macro_route_progress          # guide arc along current locked route (m)
float64 macro_guide_lookahead         # arc lookahead used (m)
string  macro_guide_update_reason     # why the guide was re-selected
float64 macro_no_progress_duration    # continuous no-translation time in macro (s)
