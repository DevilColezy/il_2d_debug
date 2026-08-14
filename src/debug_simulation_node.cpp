// ===================================================================
//  debug_simulation_node.cpp
//
//  ROS adaptation of DebugSimulation:
//    * one 30 Hz driver timer (the ONLY wall timer; the 5 Hz macro runs
//      inside the core at tick % 6 == 0 — no drifting timers);
//    * topics for the GUI/RViz;
//    * services for pause / deterministic stepping / task & scene
//      generation / navigation-goal changes / speed.
//
//  While paused the timer does nothing, so pause can never be bypassed
//  by the wall clock.  Single-step services advance exactly the
//  requested ticks under a mutex.
// ===================================================================

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "il_2d_multiscale_debug/debug_simulation.hpp"
#include "il_2d_multiscale_debug/params_io.hpp"
#include "il_2d_multiscale_debug/DebugSnapshot.h"
#include "il_2d_multiscale_debug/ExportFlightLog.h"
#include "il_2d_multiscale_debug/PlannerCandidate.h"
#include "il_2d_multiscale_debug/GenerateScene.h"
#include "il_2d_multiscale_debug/GenerateTask.h"
#include "il_2d_multiscale_debug/ResetTask.h"
#include "il_2d_multiscale_debug/SetNavigationGoal.h"
#include "il_2d_multiscale_debug/SetPaused.h"
#include "il_2d_multiscale_debug/SetSimSpeed.h"
#include "il_2d_multiscale_debug/StepSimulation.h"

using namespace il_2d_multiscale_debug;

namespace {

std_msgs::Header header() {
    std_msgs::Header h;
    h.stamp = ros::Time::now();
    h.frame_id = "world";
    return h;
}

}  // namespace

class SimNode {
public:
    SimNode(const Params2D& p, ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : p_(p), nh_(nh), pnh_(pnh), sim_(p) {
        // seed is read with readUint64Param (string or non-negative int;
        // seed 0 is a VALID seed).  The launch passes it as a string.
        if (!readUint64Param(pnh_, "seed", seed_param_)) {
            ROS_WARN_STREAM("seed param missing/unparsable; using default "
                            << p_.default_seed);
            seed_param_ = p_.default_seed;
        }
        pnh_.param("paused", paused_, true);
        pnh_.param("show_truth", show_truth_, true);
        pnh_.param("sim_speed", sim_speed_, p_.gui_default_speed);

        // Startup housekeeping: remove stale flight logs from the program's
        // own log directory (only debug_simulation_node runs this; the GUI
        // node never cleans).  Safe, non-recursive, first level only.
        clearOldFlightLogs();

        pub_snapshot_ = nh_.advertise<DebugSnapshot>("debug_snapshot", 10);
        pub_obs_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_observation", 2);
        pub_patch_ = nh_.advertise<nav_msgs::OccupancyGrid>("instantaneous_patch", 2);
        pub_esdf_ = nh_.advertise<nav_msgs::OccupancyGrid>("truth_esdf", 1, true);
        pub_selectable_ = nh_.advertise<nav_msgs::OccupancyGrid>("selectable_mask", 1, true);
        pub_local_plan_ = nh_.advertise<nav_msgs::Path>("local_plan", 2);
        pub_executed_ = nh_.advertise<nav_msgs::Path>("executed_path", 2);
        pub_left_ = nh_.advertise<nav_msgs::Path>("left_privileged_path", 1, true);
        pub_right_ = nh_.advertise<nav_msgs::Path>("right_privileged_path", 1, true);
        pub_locked_ = nh_.advertise<nav_msgs::Path>("locked_route", 1, true);
        pub_rejected_ = nh_.advertise<visualization_msgs::MarkerArray>("rejected_candidates", 2);
        pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("debug_markers", 2);
        pub_obstacles_ = nh_.advertise<visualization_msgs::MarkerArray>("scene_obstacles", 1, true);

        srv_pause_ = nh_.advertiseService("set_paused", &SimNode::cbSetPaused, this);
        srv_step_ = nh_.advertiseService("step_simulation", &SimNode::cbStep, this);
        srv_reset_ = nh_.advertiseService("reset_task", &SimNode::cbResetTask, this);
        srv_new_scene_ = nh_.advertiseService("generate_scene", &SimNode::cbGenerateScene, this);
        srv_new_task_ = nh_.advertiseService("generate_task", &SimNode::cbGenerateTask, this);
        srv_goal_ = nh_.advertiseService("set_navigation_goal", &SimNode::cbSetGoal, this);
        srv_speed_ = nh_.advertiseService("set_sim_speed", &SimNode::cbSetSpeed, this);
        srv_export_log_ = nh_.advertiseService(
            "export_flight_log", &SimNode::cbExportFlightLog, this);

        sim_.setPaused(paused_);
        std::string reason;
        if (!sim_.newScene(seed_param_, reason)) {
            // The user-specified seed is NEVER silently replaced.  Report
            // clearly and leave the sim uninitialized (honest snapshot only).
            ROS_ERROR_STREAM("Initial scene generation FAILED for seed "
                             << seed_param_ << ": " << reason
                             << ".  Simulation NOT initialized — call "
                                "/generate_scene with another seed to commit one.");
        }
        publishAll();

        const double period = (1.0 / 30.0) / std::max(0.05, sim_speed_);
        timer_ = nh_.createTimer(ros::Duration(period), &SimNode::timerCb, this);
    }

private:
    // ── Timer (the ONLY wall clock; no-op while paused) ─────────────
    void timerCb(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (paused_) return;
        if (sim_.step()) publishAll();
    }

    // ── Services ────────────────────────────────────────────────────
    bool cbSetPaused(SetPaused::Request& req, SetPaused::Response& res) {
        std::lock_guard<std::mutex> lock(mtx_);
        paused_ = req.paused;
        sim_.setPaused(paused_);
        publishAll();
        fillRes(res, true, req.paused ? "ok; paused" : "ok; running");
        return true;
    }

    bool cbStep(StepSimulation::Request& req, StepSimulation::Response& res) {
        std::lock_guard<std::mutex> lock(mtx_);
        // ATOMIC single-step: stop auto advance immediately (we hold the
        // mutex, so the wall timer cannot insert); the service UNCONDITION-
        // ALLY leaves the simulation paused when it returns.
        paused_ = true;
        sim_.setPaused(true);
        if (!sim_.initialized()) {
            publishAll();
            fillRes(res, false, "simulation not initialized");
            return true;
        }
        if (req.steps == 0 && !req.step_to_next_5hz) {
            publishAll();
            fillRes(res, false,
                    "nothing requested (steps==0 and step_to_next_5hz==false)");
            return true;
        }
        sim_.setPaused(false);  // allow the core to advance under our mutex
        if (req.step_to_next_5hz) {
            if (!sim_.stepToNext5Hz()) {
                sim_.setPaused(true);
                paused_ = true;
                publishAll();
                fillRes(res, false, "terminal — nothing advanced");
                return true;
            }
        } else {
            uint32_t advanced = 0;
            for (uint32_t i = 0; i < req.steps; ++i) {
                if (!sim_.step()) break;
                ++advanced;
            }
            if (advanced != req.steps) {
                sim_.setPaused(true);
                paused_ = true;
                publishAll();
                fillRes(res, false,
                        "terminal before requested 30Hz steps completed (advanced=" +
                            std::to_string(advanced) + ")");
                return true;
            }
        }
        // Unconditionally remain paused — never restore a previous Run.
        sim_.setPaused(true);
        paused_ = true;
        publishAll();
        fillRes(res, true, "ok; simulation remains paused");
        return true;
    }

    bool cbResetTask(ResetTask::Request& req, ResetTask::Response& res) {
        std::lock_guard<std::mutex> lock(mtx_);
        // Reset restarts the CURRENT, IDENTICAL task (same scene/start/
        // goal/yaw/task_id).  req.seed / req.use_default_seed are retained
        // for interface compatibility and are NOT used by reset — sample a
        // new task via /generate_task instead.
        (void)req;
        std::string reason;
        if (!sim_.resetTask(reason)) {
            fillRes(res, false, "reset failed: " + reason);
            return true;
        }
        publishAll();
        fillRes(res, true, "ok; current task restarted");
        return true;
    }

    bool cbGenerateTask(GenerateTask::Request& req, GenerateTask::Response& res) {
        std::lock_guard<std::mutex> lock(mtx_);
        std::string reason;
        if (!sim_.newTaskInSameScene(req.seed, req.use_default_seed, reason)) {
            fillRes(res, false, "task sampling failed: " + reason);
            return true;
        }
        publishAll();
        fillRes(res, true, "ok; task sampled");
        return true;
    }

    bool cbGenerateScene(GenerateScene::Request& req, GenerateScene::Response& res) {
        std::lock_guard<std::mutex> lock(mtx_);
        // use_default_seed → the launch `seed` param; otherwise `seed` is
        // used verbatim (0 is a legal scene seed).
        const uint64_t seed = req.use_default_seed ? seed_param_ : req.seed;
        std::string reason;
        if (!sim_.newScene(seed, reason)) {
            fillRes(res, false, "scene generation failed: " + reason);
            return true;
        }
        publishAll();
        fillRes(res, true,
                "ok; scene generated (scene_id=" + std::to_string(sim_.sceneId()) +
                    ", seed=" + std::to_string(seed) + ")");
        return true;
    }

    bool cbSetGoal(SetNavigationGoal::Request& req, SetNavigationGoal::Response& res) {
        std::lock_guard<std::mutex> lock(mtx_);
        std::string reason;
        if (!sim_.setNavigationGoal(Vec2d(req.goal[0], req.goal[1]), reason)) {
            fillRes(res, false, "goal rejected: " + reason);
            return true;
        }
        publishAll();
        // Success is a PENDING goal — it becomes the real final goal at the
        // next 5 Hz boundary (this is a SUCCESS, not a failure).
        fillRes(res, true, "goal pending; will be accepted at next 5Hz boundary");
        return true;
    }

    bool cbSetSpeed(SetSimSpeed::Request& req, SetSimSpeed::Response& res) {
        std::lock_guard<std::mutex> lock(mtx_);
        sim_speed_ = std::max(0.05, req.speed);
        timer_.setPeriod(ros::Duration((1.0 / 30.0) / sim_speed_));
        fillRes(res, true, "ok; speed set");
        return true;
    }

    static std::string csvString(const std::string& value) {
        std::string escaped;
        escaped.reserve(value.size() + 2);
        escaped.push_back('"');
        for (const char ch : value) {
            if (ch == '"') escaped.push_back('"');
            escaped.push_back(ch);
        }
        escaped.push_back('"');
        return escaped;
    }

    /// Resolve a log path.  An EMPTY request directory falls back to the
    /// configured `logging/output_directory` (never a second hard-coded
    /// default); `~` / `~/...` are expanded via $HOME.
    std::filesystem::path expandUserPath(const std::string& raw) const {
        const std::string use = raw.empty() ? p_.logging_output_directory : raw;
        if (use == "~" || use.rfind("~/", 0) == 0) {
            const char* user_home = std::getenv("HOME");
            if (user_home && *user_home) {
                return use == "~" ? std::filesystem::path(user_home)
                                   : std::filesystem::path(user_home) / use.substr(2);
            }
            return std::filesystem::path{};
        }
        return std::filesystem::path(use);
    }

    /// True only for an explicitly configured path that is safe to run a
    /// first-level cleanup in: never empty, never the filesystem root, never
    /// the user's HOME itself, never `.`/`..`.
    static bool isSafeCleanupDirectory(const std::filesystem::path& dir) {
        if (dir.empty()) return false;
        const std::filesystem::path norm = dir.lexically_normal();
        if (norm.empty() || norm == "." || norm == "..") return false;
        if (norm == norm.root_path()) return false;
        std::error_code cwd_ec;
        const std::filesystem::path cwd_raw =
            std::filesystem::current_path(cwd_ec);
        if (!cwd_ec) {
            const std::filesystem::path cwd =
                std::filesystem::weakly_canonical(cwd_raw, cwd_ec);
            if (!cwd_ec && !cwd.empty() && norm == cwd) return false;
        }
        const char* user_home = std::getenv("HOME");
        if (user_home && *user_home) {
            std::error_code home_ec;
            const std::filesystem::path home =
                std::filesystem::weakly_canonical(
                    std::filesystem::path(user_home), home_ec);
            if ((!home_ec && norm == home) ||
                norm == std::filesystem::path(user_home).lexically_normal()) {
                return false;
            }
        }
        return true;
    }

    /// Delete ONLY first-level regular files named `prefix*.csv` inside the
    /// configured log directory.  Never recursive; never deletes directories,
    /// symlinks or other files; refuses empty/root/HOME/wild paths with a
    /// ROS_WARN (never crashes); reports the removed count.  Runs exactly once
    /// per debug_simulation_node start when logging/clear_on_start is true.
    void clearOldFlightLogs() {
        if (!p_.logging_clear_on_start) return;
        const std::filesystem::path configured_dir =
            expandUserPath(p_.logging_output_directory);
        if (!isSafeCleanupDirectory(configured_dir) ||
            p_.logging_filename_prefix.empty()) {
            ROS_WARN_STREAM("Refusing to clear flight logs: unsafe directory "
                            "or empty filename prefix; logging/output_directory '"
                            << p_.logging_output_directory << "' (resolved to "
                            << configured_dir << ")");
            return;
        }
        std::error_code ec;
        std::filesystem::create_directories(configured_dir, ec);
        if (ec) {
            ROS_WARN_STREAM("Cannot create log directory " << configured_dir << ": "
                          << ec.message());
            return;
        }
        std::error_code dsec;
        const auto dir_status =
            std::filesystem::symlink_status(configured_dir, dsec);
        if (dsec || std::filesystem::is_symlink(dir_status) ||
            !std::filesystem::is_directory(dir_status)) {
            ROS_WARN_STREAM("Refusing to clear flight logs: directory is "
                            "invalid or a symlink: " << configured_dir);
            return;
        }
        // Canonicalise after creation so `..` and any symlinked parent
        // components are resolved before the broad-path safety check and
        // before iteration.  This prevents a lexically harmless path from
        // resolving to HOME or the filesystem root.
        std::error_code cec;
        const std::filesystem::path dir =
            std::filesystem::weakly_canonical(configured_dir, cec);
        if (cec || !isSafeCleanupDirectory(dir)) {
            ROS_WARN_STREAM("Refusing to clear flight logs: cannot safely "
                            "canonicalise " << configured_dir << " ("
                            << cec.message() << ")");
            return;
        }
        std::error_code itec;
        std::filesystem::directory_iterator it(dir, itec);
        const std::filesystem::directory_iterator end;
        if (itec) {
            ROS_WARN_STREAM("Cannot open log directory " << dir << ": "
                          << itec.message());
            return;
        }
        std::size_t removed = 0;
        for (; it != end; it.increment(itec)) {
            if (itec) break;
            const std::filesystem::path p = it->path();
            std::error_code sec;
            const auto status = std::filesystem::symlink_status(p, sec);
            if (sec || !std::filesystem::is_regular_file(status)) {
                // directory / symlink / other file (or stat error) — never
                // deleted; a stat failure is reported but does not crash.
                if (sec) {
                    ROS_WARN_STREAM("Stat failed " << p << ": " << sec.message());
                }
                continue;
            }
            const std::string name = p.filename().string();
            if (name.rfind(p_.logging_filename_prefix, 0) != 0) continue;
            if (p.extension() != ".csv") continue;
            std::error_code rec;
            if (!std::filesystem::remove(p, rec)) {
                ROS_WARN_STREAM("Failed to remove old flight log " << p << ": "
                              << rec.message());
            } else {
                ++removed;
            }
        }
        if (itec) {
            ROS_WARN_STREAM("Flight-log cleanup in " << dir << " interrupted: "
                          << itec.message());
        }
        ROS_INFO_STREAM("Cleared " << removed << " old flight log(s) in " << dir);
    }

    bool cbExportFlightLog(ExportFlightLog::Request& req,
                           ExportFlightLog::Response& res) {
        std::lock_guard<std::mutex> lock(mtx_);
        res.scene_id = sim_.sceneId();
        res.task_id = sim_.taskId();
        res.row_count = 0;

        if (!sim_.initialized()) {
            res.success = false;
            res.reason = "simulation not initialized";
            return true;
        }
        const auto& rows = sim_.flightLog();
        if (rows.empty()) {
            res.success = false;
            res.reason = "current task log is empty";
            return true;
        }

        const std::filesystem::path directory =
            expandUserPath(req.output_directory);
        std::error_code ec;
        std::filesystem::create_directories(directory, ec);
        if (ec) {
            res.success = false;
            res.reason = "cannot create output directory: " + ec.message();
            return true;
        }

        std::ostringstream filename;
        filename << p_.logging_filename_prefix << "scene_" << sim_.sceneId()
                 << "_task_" << sim_.taskId()
                 << "_tick_" << sim_.tick()
                 << "_" << ros::WallTime::now().toNSec() << ".csv";
        const std::filesystem::path output_path = directory / filename.str();
        std::ofstream out(output_path, std::ios::out | std::ios::trunc);
        if (!out) {
            res.success = false;
            res.reason = "cannot open output file";
            return true;
        }
        out << std::setprecision(17);

        const Scene2D& scene = sim_.scene();
        const Task2D& task = sim_.task();
        out << "# format,il_2d_multiscale_debug_flight_log_v7\n";
        out << "# scene_id," << scene.scene_id << "\n";
        out << "# task_id," << task.task_id << "\n";
        out << "# scene_seed," << scene.seed << "\n";
        out << "# task_seed," << task.seed << "\n";
        out << "# bounds," << scene.min_bounds.x() << ',' << scene.min_bounds.y()
            << ',' << scene.max_bounds.x() << ',' << scene.max_bounds.y() << "\n";
        const Vec2d initial_goal = rows.front().final_goal;
        out << "# initial_task," << task.start.x() << ',' << task.start.y() << ','
            << task.initial_yaw << ',' << initial_goal.x() << ',' << initial_goal.y()
            << "\n";
        // Goal semantics: the ORIGINAL sampled goal (never overwritten by
        // clicks), the goal live after the episode's FIRST 5 Hz acceptance
        // (== original when no pending goal existed at tick 0), and the
        // final goal at export time.  Per-row final/pending/local targets
        // remain in the data columns below.
        const Vec2d orig_goal = sim_.originalSampledGoal();
        const Vec2d init_goal = sim_.initialAcceptedGoal();
        out << "# original_sampled_goal," << orig_goal.x() << ',' << orig_goal.y()
            << "\n";
        out << "# initial_accepted_goal," << init_goal.x() << ',' << init_goal.y()
            << "\n";
        out << "# final_goal_at_export," << task.goal.x() << ',' << task.goal.y()
            << "\n";
        out << "# obstacle_count," << scene.obstacles.size() << "\n";
        for (const Obstacle2D& obstacle : scene.obstacles) {
            out << "# obstacle," << obstacle.id << ',' << obstacle.center.x() << ','
                << obstacle.center.y() << ',' << obstacle.radius << "\n";
        }

        out << "tick,processing_tick,time_s,vehicle_x,vehicle_y,yaw,"
               "velocity_world_x,velocity_world_y,vehicle_yaw_rate,"
               "final_goal_x,final_goal_y,local_target_x,local_target_y,"
               "local_target_valid,local_target_is_macro_guide,"
               "pending_goal_set,pending_goal_revision,pending_goal_x,pending_goal_y,"
               "cmd_vx_body,cmd_vy_body,cmd_yaw_rate,planner_success,"
               "planner_failure_reason,turn_mode,blocked_observed,"
               "immediate_avoidance,emergency_brake,selected_trajectory_points,"
               "rejected_candidate_count,fsm_state,previous_fsm_state,"
               "transition_tick,transition_reason,macro_active,side,side_reason,"
               "side_left_score,side_right_score,consecutive_failures_30hz,"
               "macro_stable_exit_count,local_target_update_event,macro_tick_event,"
               "accepted_goal_event,local_target_updated_this_tick,"
               "macro_tick_ran_this_tick,blocker_id,blocker_association,"
               "blocker_passed_latched,entry_vehicle_progress,entry_blocker_progress,"
               "entry_projection_dist,entry_segment_index,entry_progress_delta,"
               "entry_progress_max_delta,min_observed_clearance,truth_min_clearance,"
               "goal_reached,collision,task_invalid,used_truth_by_local_planner,"
               "used_global_esdf_by_local_planner,used_global_path_by_local_planner,"
               "macro_used_privileged_esdf,side_selected_from_visible_evidence,"
               "side_selected_using_current_patch,side_ambiguous_defaulted_right,"
               "macro_enter_event,macro_exit_event,obstacle_first_observed_event,"
               "immediate_avoidance_event,emergency_brake_event,"
               "reject_not_known_free,reject_outside_current_fov,"
               "reject_observed_clearance_too_small,reject_no_progress,"
               "reject_other,unknown_recovery_ticks,unknown_recovery_active,"
               "unknown_recovery_episode_count,"
               "target_bearing_error_deg,selected_terminal_heading_error_deg,"
               "selected_velocity_alignment_error_deg,selected_cross_track_error_m,"
               "selected_cost_total,selected_cost_progress,selected_cost_clearance,"
               "selected_cost_smoothness,selected_cost_speed_change,"
               "selected_cost_yaw_rate_change,selected_cost_terminal_heading,"
               "selected_cost_velocity_alignment,selected_cost_cross_track,"
               "selected_soft_min_clearance_m,selected_dynamic_required_clearance_m,"
               "selected_closing_speed_mps,handoff_clearance_m,"
               "dynamic_clearance_blocked,local_limit_cycle_detected,"
               "dynamic_window_candidate_count,reject_insufficient_braking_clearance,"
               "start_clearance_recovery_used,"
               "selected_intent_vx_body,selected_intent_vy_body,selected_intent_yaw_rate,"
               "selected_output_vx_body,selected_output_vy_body,selected_output_yaw_rate,"
               "nominal_progress_m,executable_progress_m,safe_prefix_duration_s,"
               "progress_qualified,planner_status,planner_status_name,"
               "stationary_candidate_selected,stationary_selection_reason,"
               "target_discontinuity_reset,target_mission_revision,"
               "candidate_progress_qualified,output_progress_qualified,"
               "current_observed_clearance,current_truth_clearance,"
               // ── v7: early-avoidance risk + rolling macro guide ──
               "selected_cost_obstacle_risk,local_corridor_blocked,"
               "first_blocking_obstacle_distance,predicted_closest_clearance,"
               "time_to_collision,obstacle_risk_cost,avoidance_strength,"
               "avoidance_active,local_target_distance,"
               "macro_route_progress,macro_guide_lookahead,"
               "macro_guide_update_reason,macro_no_progress_duration\n";

        for (const FlightLogSample& row : rows) {
            const SimSnapshot& s = row.snapshot;
            out << s.tick << ',' << s.processing_tick << ',' << s.time << ','
                << s.vehicle_position.x() << ',' << s.vehicle_position.y() << ','
                << s.yaw << ',' << row.velocity_world.x() << ','
                << row.velocity_world.y() << ',' << row.vehicle_yaw_rate << ','
                << row.final_goal.x() << ',' << row.final_goal.y() << ','
                << s.local_target.x() << ',' << s.local_target.y() << ','
                << s.local_target_valid << ',' << s.local_target_is_macro_guide << ','
                << s.pending_goal_set << ',' << s.pending_goal_revision << ','
                << s.pending_goal_position.x() << ',' << s.pending_goal_position.y() << ','
                << row.cmd_vx_body << ',' << row.cmd_vy_body << ','
                << row.cmd_yaw_rate << ',' << s.planner_success << ','
                << csvString(s.planner_failure_reason) << ',' << row.turn_mode << ','
                << row.blocked_observed << ',' << row.immediate_avoidance << ','
                << row.emergency_brake << ',' << row.selected_trajectory_points << ','
                << row.rejected_candidate_count << ','
                << csvString(fsmStateName(s.fsm_state)) << ','
                << csvString(fsmStateName(s.prev_fsm_state)) << ','
                << s.transition_tick << ',' << csvString(s.transition_reason) << ','
                << s.macro_active << ',' << csvString(sideName(s.side)) << ','
                << csvString(s.side_reason) << ',' << s.side_left_score << ','
                << s.side_right_score << ',' << s.consecutive_failures_30hz << ','
                << s.macro_stable_exit_count << ',' << s.local_target_update_event << ','
                << s.macro_tick_event << ',' << s.accepted_goal_event << ','
                << s.local_target_updated_this_tick << ','
                << s.macro_tick_ran_this_tick << ',' << s.blocker_id << ','
                << csvString(blockerAssociationName(s.blocker_association)) << ','
                << s.blocker_passed_latched << ',' << s.entry_vehicle_progress << ','
                << s.entry_blocker_progress << ',' << s.entry_projection_dist << ','
                << s.entry_segment_index << ',' << s.entry_progress_delta << ','
                << s.entry_progress_max_delta << ',' << s.min_observed_clearance << ','
                << s.truth_min_clearance << ',' << s.goal_reached << ',' << s.collision
                << ',' << s.task_invalid << ','
                << s.audit.used_truth_by_local_planner << ','
                << s.audit.used_global_esdf_by_local_planner << ','
                << s.audit.used_global_path_by_local_planner << ','
                << s.audit.macro_used_privileged_esdf << ','
                << s.audit.side_selected_from_visible_evidence << ','
                << s.audit.side_selected_using_current_patch << ','
                << s.audit.side_ambiguous_defaulted_right << ','
                << s.audit.macro_enter_event << ',' << s.audit.macro_exit_event << ','
                << s.audit.obstacle_first_observed_event << ','
                << s.audit.immediate_avoidance_event << ','
                << s.audit.emergency_brake_event << ','
                << s.reject_not_known_free << ',' << s.reject_outside_current_fov
                << ',' << s.reject_observed_clearance_too_small << ','
                << s.reject_no_progress << ',' << s.reject_other << ','
                << s.unknown_recovery_ticks << ',' << s.unknown_recovery_active
                << ',' << s.unknown_recovery_episode_count << ','
                << s.target_bearing_error_deg << ','
                << s.selected_terminal_heading_error_deg << ','
                << s.selected_velocity_alignment_error_deg << ','
                << s.selected_cross_track_error_m << ',' << s.selected_cost_total
                << ',' << s.selected_cost_progress << ',' << s.selected_cost_clearance
                << ',' << s.selected_cost_smoothness << ',' << s.selected_cost_speed_change
                << ',' << s.selected_cost_yaw_rate_change << ','
                << s.selected_cost_terminal_heading << ','
                << s.selected_cost_velocity_alignment << ','
                << s.selected_cost_cross_track << ','
                << s.selected_soft_min_clearance_m << ','
                << s.selected_dynamic_required_clearance_m << ','
                << s.selected_closing_speed_mps << ','
                << s.handoff_clearance_m << ','
                << s.dynamic_clearance_blocked << ','
                << s.local_limit_cycle_detected << ','
                << s.dynamic_window_candidate_count << ','
                << s.reject_insufficient_braking_clearance << ','
                << s.start_clearance_recovery_used << ','
                // ── v5: intent / output / progress / status ─────────
                << s.selected_intent_vx_body << ','
                << s.selected_intent_vy_body << ','
                << s.selected_intent_yaw_rate << ','
                << s.selected_output_vx_body << ','
                << s.selected_output_vy_body << ','
                << s.selected_output_yaw_rate << ','
                << s.nominal_progress_m << ',' << s.executable_progress_m << ','
                << s.safe_prefix_duration_s << ',' << s.progress_qualified << ','
                << static_cast<int>(s.planner_status) << ','
                << csvString(s.planner_status_name) << ','
                << s.stationary_candidate_selected << ','
                << csvString(s.stationary_selection_reason) << ','
                << s.target_discontinuity_reset << ','
                << s.target_mission_revision << ','
                // v6: distinguish candidate properties from the final
                // supervised output and instantaneous from cumulative
                // clearance.  Appended so all v1-v5 columns stay stable.
                << s.candidate_progress_qualified << ','
                << s.output_progress_qualified << ','
                << s.current_observed_clearance << ','
                << s.current_truth_clearance << ','
                // ── v7: early-avoidance risk + rolling macro guide ──
                << s.selected_cost_obstacle_risk << ','
                << s.local_corridor_blocked << ','
                << s.first_blocking_obstacle_distance << ','
                << s.predicted_closest_clearance << ','
                << s.time_to_collision << ','
                << s.obstacle_risk_cost << ','
                << s.avoidance_strength << ','
                << s.avoidance_active << ','
                << s.local_target_distance << ','
                << s.macro_route_progress << ','
                << s.macro_guide_lookahead << ','
                << csvString(s.macro_guide_update_reason) << ','
                << s.macro_no_progress_duration << '\n';
        }
        out.close();
        if (!out) {
            res.success = false;
            res.reason = "failed while writing output file";
            return true;
        }

        res.success = true;
        res.path = output_path.string();
        res.row_count = rows.size();
        res.reason = "exported " + std::to_string(rows.size()) +
                     " rows to " + res.path;
        ROS_INFO_STREAM(res.reason);
        return true;
    }

    /// Explicit success/reason fill for every service response type.
    template <typename Resp>
    void fillRes(Resp& r, bool success, const std::string& reason) {
        r.success = success;
        r.reason = reason;
        r.tick = sim_.tick();
        r.scene_id = sim_.sceneId();
        r.task_id = sim_.taskId();
    }

    // ── Publishing ──────────────────────────────────────────────────
    void publishAll() {
        if (!sim_.initialized()) {
            // Do NOT publish fake truth grids / paths / markers.  Publish
            // only an honest snapshot (simulation_initialized=false).
            publishSnapshot();
            return;
        }
        publishSnapshot();
        publishObservation();
        publishPatch();
        // Latched truth: republish when the scene, the task, the formally
        // accepted goal revision, or the pending-goal state changed (the
        // final-goal markers must move when a GUI click is accepted, even
        // though task_id stays the same).  With show_truth:=false nothing
        // truth-like is published at all.
        if (show_truth_) {
            const uint64_t gr = sim_.acceptedGoalEvent();
            const bool pending = sim_.hasPendingGoal();
            const uint64_t pr = sim_.pendingGoalRevision();
            const bool grid_change = sim_.sceneId() != last_scene_id_;
            // Republish the task/pending markers when the task changed, a
            // goal was formally accepted, the pending FLAG changed, or the
            // pending REVISION changed (a new click while a goal is already
            // pending must move the pending marker even though pending==true
            // both before and after).
            const bool task_change = sim_.taskId() != last_task_id_ ||
                                     gr != last_goal_revision_ ||
                                     pending != last_pending_ ||
                                     pr != last_pending_revision_;
            if (grid_change) publishTruthGrids();
            if (grid_change || task_change) publishMarkersObstacles();
            last_scene_id_ = sim_.sceneId();
            last_task_id_ = sim_.taskId();
            last_goal_revision_ = gr;
            last_pending_ = pending;
            last_pending_revision_ = pr;
        }
        publishPaths();
        publishCandidates();
        publishMarkers();
    }

    void publishSnapshot() {
        const SimSnapshot& s = sim_.snapshot();
        DebugSnapshot m;
        m.header = header();
        m.scene_id = s.scene_id;
        m.task_id = s.task_id;
        m.seed = s.seed;
        m.tick = s.tick;
        m.processing_tick = s.processing_tick;
        m.time = s.time;
        m.simulation_initialized = sim_.initialized();  // authoritative (also pre-init)
        m.paused = sim_.paused();  // authoritative pause state from the core
        m.fsm_state = static_cast<uint8_t>(s.fsm_state);
        m.fsm_state_name = fsmStateName(s.fsm_state);
        m.previous_fsm_state = static_cast<uint8_t>(s.prev_fsm_state);
        m.previous_fsm_state_name = fsmStateName(s.prev_fsm_state);
        m.transition_tick = s.transition_tick;
        m.transition_reason = s.transition_reason;
        m.macro_active = s.macro_active;
        m.side = static_cast<uint8_t>(s.side);
        m.side_reason = s.side_reason;
        m.blocker_association = static_cast<uint8_t>(s.blocker_association);
        m.blocker_association_name = blockerAssociationName(s.blocker_association);
        m.blocker_passed_latched = s.blocker_passed_latched;
        m.start_clearance_recovery_used = s.start_clearance_recovery_used;
        m.entry_vehicle_progress = s.entry_vehicle_progress;
        m.entry_blocker_progress = s.entry_blocker_progress;
        m.entry_projection_dist = s.entry_projection_dist;
        m.entry_segment_index = s.entry_segment_index;
        m.entry_progress_delta = s.entry_progress_delta;
        m.entry_progress_max_delta = s.entry_progress_max_delta;
        m.planner_success = s.planner_success;
        m.planner_failure_reason = s.planner_failure_reason;
        m.selected_soft_min_clearance_m = s.selected_soft_min_clearance_m;
        m.selected_dynamic_required_clearance_m =
            s.selected_dynamic_required_clearance_m;
        m.selected_closing_speed_mps = s.selected_closing_speed_mps;
        m.handoff_clearance_m = s.handoff_clearance_m;
        m.dynamic_clearance_blocked = s.dynamic_clearance_blocked;
        m.local_limit_cycle_detected = s.local_limit_cycle_detected;
        m.dynamic_window_candidate_count = s.dynamic_window_candidate_count;
        m.selected_intent_vx_body = s.selected_intent_vx_body;
        m.selected_intent_vy_body = s.selected_intent_vy_body;
        m.selected_intent_yaw_rate = s.selected_intent_yaw_rate;
        m.selected_output_vx_body = s.selected_output_vx_body;
        m.selected_output_vy_body = s.selected_output_vy_body;
        m.selected_output_yaw_rate = s.selected_output_yaw_rate;
        m.selected_output_speed_mps = s.selected_output_speed_mps;
        m.nominal_progress_m = s.nominal_progress_m;
        m.executable_progress_m = s.executable_progress_m;
        m.safe_prefix_duration_s = s.safe_prefix_duration_s;
        m.candidate_progress_qualified = s.candidate_progress_qualified;
        m.output_progress_qualified = s.output_progress_qualified;
        m.progress_qualified = s.progress_qualified;
        m.planner_status = static_cast<uint8_t>(s.planner_status);
        m.planner_status_name = s.planner_status_name;
        m.stationary_candidate_selected = s.stationary_candidate_selected;
        m.stationary_selection_reason = s.stationary_selection_reason;
        m.target_discontinuity_reset = s.target_discontinuity_reset;
        m.target_mission_revision = s.target_mission_revision;
        m.selected_cost_obstacle_risk = s.selected_cost_obstacle_risk;
        // Early-avoidance risk diagnostics (v7).
        m.local_corridor_blocked = s.local_corridor_blocked;
        m.first_blocking_obstacle_distance =
            s.first_blocking_obstacle_distance;
        m.predicted_closest_clearance = s.predicted_closest_clearance;
        m.time_to_collision = s.time_to_collision;
        m.obstacle_risk_cost = s.obstacle_risk_cost;
        m.avoidance_strength = s.avoidance_strength;
        m.avoidance_active = s.avoidance_active;
        m.local_target_distance = s.local_target_distance;
        // Rolling macro guide diagnostics (v7).
        m.macro_route_progress = s.macro_route_progress;
        m.macro_guide_lookahead = s.macro_guide_lookahead;
        m.macro_guide_update_reason = s.macro_guide_update_reason;
        m.macro_no_progress_duration = s.macro_no_progress_duration;
        m.target_bearing_error_deg = s.target_bearing_error_deg;
        m.selected_terminal_heading_error_deg =
            s.selected_terminal_heading_error_deg;
        m.selected_velocity_alignment_error_deg =
            s.selected_velocity_alignment_error_deg;
        m.selected_cross_track_error_m = s.selected_cross_track_error_m;
        m.selected_cost_total = s.selected_cost_total;
        m.selected_cost_progress = s.selected_cost_progress;
        m.selected_cost_clearance = s.selected_cost_clearance;
        m.selected_cost_smoothness = s.selected_cost_smoothness;
        m.selected_cost_speed_change = s.selected_cost_speed_change;
        m.selected_cost_yaw_rate_change = s.selected_cost_yaw_rate_change;
        m.selected_cost_terminal_heading = s.selected_cost_terminal_heading;
        m.selected_cost_velocity_alignment = s.selected_cost_velocity_alignment;
        m.selected_cost_cross_track = s.selected_cost_cross_track;
        m.consecutive_failures_30hz = s.consecutive_failures_30hz;
        m.macro_stable_exit_count = s.macro_stable_exit_count;
        m.reject_not_known_free = s.reject_not_known_free;
        m.reject_outside_current_fov = s.reject_outside_current_fov;
        m.reject_observed_clearance_too_small =
            s.reject_observed_clearance_too_small;
        m.reject_no_progress = s.reject_no_progress;
        m.reject_other = s.reject_other;
        m.reject_insufficient_braking_clearance =
            s.reject_insufficient_braking_clearance;
        m.rejected_candidate_count = s.rejected_candidate_count;
        m.unknown_recovery_ticks = s.unknown_recovery_ticks;
        m.unknown_recovery_active = s.unknown_recovery_active;
        m.unknown_recovery_episode_count = s.unknown_recovery_episode_count;
        m.local_target_update_event = s.local_target_update_event;
        m.macro_tick_event = s.macro_tick_event;
        m.blocker_id = s.blocker_id;
        m.accepted_goal_event = s.accepted_goal_event;
        m.macro_reentry_guard_ticks = s.reentry_guard_ticks;
        m.local_target_updated_this_tick = s.local_target_updated_this_tick;
        m.macro_tick_ran_this_tick = s.macro_tick_ran_this_tick;
        m.pending_goal_set = s.pending_goal_set;
        m.pending_goal_revision = s.pending_goal_revision;
        m.pending_goal_position[0] = s.pending_goal_position.x();
        m.pending_goal_position[1] = s.pending_goal_position.y();
        m.min_observed_clearance = s.min_observed_clearance;
        m.truth_min_clearance = s.truth_min_clearance;
        m.current_observed_clearance = s.current_observed_clearance;
        m.current_truth_clearance = s.current_truth_clearance;
        m.goal_reached = s.goal_reached;
        m.collision = s.collision;
        m.task_invalid = s.task_invalid;
        m.used_truth_by_local_planner = s.audit.used_truth_by_local_planner;
        m.used_global_esdf_by_local_planner = s.audit.used_global_esdf_by_local_planner;
        m.used_global_path_by_local_planner = s.audit.used_global_path_by_local_planner;
        m.macro_used_privileged_esdf = s.audit.macro_used_privileged_esdf;
        m.side_selected_from_visible_evidence = s.audit.side_selected_from_visible_evidence;
        m.side_ambiguous_defaulted_right = s.audit.side_ambiguous_defaulted_right;
        m.side_selected_using_current_patch = s.audit.side_selected_using_current_patch;
        m.side_left_score = s.side_left_score;
        m.side_right_score = s.side_right_score;
        m.macro_enter_event = s.audit.macro_enter_event;
        m.macro_exit_event = s.audit.macro_exit_event;
        m.obstacle_first_observed_event = s.audit.obstacle_first_observed_event;
        m.immediate_avoidance_event = s.audit.immediate_avoidance_event;
        m.emergency_brake_event = s.audit.emergency_brake_event;
        m.vehicle_position[0] = s.vehicle_position.x();
        m.vehicle_position[1] = s.vehicle_position.y();
        m.yaw = s.yaw;
        m.local_target[0] = s.local_target.x();
        m.local_target[1] = s.local_target.y();
        m.local_target_valid = s.local_target_valid;
        m.local_target_is_macro_guide = s.local_target_is_macro_guide;
        pub_snapshot_.publish(m);
    }

    nav_msgs::OccupancyGrid makeGrid(const Vec2d& origin, int w, int h,
                                     double res) const {
        nav_msgs::OccupancyGrid g;
        g.header = header();
        g.info.resolution = res;
        g.info.width = static_cast<uint32_t>(w);
        g.info.height = static_cast<uint32_t>(h);
        g.info.origin.position.x = origin.x();
        g.info.origin.position.y = origin.y();
        g.info.origin.position.z = 0.0;
        g.info.origin.orientation.w = 1.0;
        g.data.assign(static_cast<size_t>(w) * h, 0);
        return g;
    }

    void publishObservation() {
        const LocalObservation& obs = sim_.observedGrid().observation();
        auto g = makeGrid(obs.origin, obs.width, obs.height, obs.resolution);
        for (size_t i = 0; i < obs.cells.size(); ++i) {
            switch (obs.cells[i]) {
                case CellState::FREE: g.data[i] = 0; break;
                case CellState::OCCUPIED: g.data[i] = 100; break;
                case CellState::UNKNOWN: g.data[i] = -1; break;
            }
        }
        pub_obs_.publish(g);
    }

    void publishPatch() {
        // INSTANTANEOUS FOV patch of the current tick (before merging) —
        // published separately so the GUI can show the semantic difference
        // between the raw sensor frame and the merged history map.
        const LocalObservation& patch = sim_.lastPatch();
        if (!patch.valid()) return;
        auto g = makeGrid(patch.origin, patch.width, patch.height, patch.resolution);
        for (size_t i = 0; i < patch.cells.size(); ++i) {
            switch (patch.cells[i]) {
                case CellState::FREE: g.data[i] = 0; break;
                case CellState::OCCUPIED: g.data[i] = 100; break;
                case CellState::UNKNOWN: g.data[i] = -1; break;
            }
        }
        pub_patch_.publish(g);
    }

    void publishTruthGrids() {
        const auto& esdf = sim_.esdf();
        auto g = makeGrid(sim_.scene().min_bounds, esdf.width(), esdf.height(),
                          esdf.resolution());
        for (int iy = 0; iy < esdf.height(); ++iy) {
            for (int ix = 0; ix < esdf.width(); ++ix) {
                g.data[static_cast<size_t>(iy) * esdf.width() + ix] =
                    static_cast<int8_t>(esdf.occupancyValueAt(ix, iy));
            }
        }
        pub_esdf_.publish(g);

        auto sel = makeGrid(sim_.scene().min_bounds, esdf.width(), esdf.height(),
                            esdf.resolution());
        const auto& conn = sim_.connectivity();
        for (int iy = 0; iy < esdf.height(); ++iy) {
            for (int ix = 0; ix < esdf.width(); ++ix) {
                const int lab = conn.labelAt(ix, iy);
                sel.data[static_cast<size_t>(iy) * esdf.width() + ix] =
                    (lab == conn.mainComponentId()) ? 100 : 0;
            }
        }
        pub_selectable_.publish(sel);
    }

    nav_msgs::Path makePath(const std::vector<Vec2d>& pts) const {
        nav_msgs::Path path;
        path.header = header();
        path.poses.reserve(pts.size());
        for (const auto& p : pts) {
            geometry_msgs::PoseStamped ps;
            ps.header = path.header;
            ps.pose.position.x = p.x();
            ps.pose.position.y = p.y();
            ps.pose.orientation.w = 1.0;
            path.poses.push_back(std::move(ps));
        }
        return path;
    }

    void publishPaths() {
        const auto& out = sim_.lastOutput();
        pub_local_plan_.publish(makePath(trajToPts(out.local.selected)));
        pub_executed_.publish(makePath(sim_.executedPath()));
        if (show_truth_) {
            // LEFT/RIGHT/locked are PRIVILEGED routes — never published
            // (nor shown) when show_truth:=false.
            pub_left_.publish(makePath(out.left_route.waypoints));
            pub_right_.publish(makePath(out.right_route.waypoints));
            pub_locked_.publish(makePath(out.locked_route.waypoints));
        }
    }

    void publishCandidates() {
        visualization_msgs::MarkerArray arr;
        // DELETEALL first: when the candidate count drops, stale markers
        // must not linger.
        visualization_msgs::Marker del;
        del.header = header();
        del.ns = "rejected";
        del.id = 0;
        del.action = visualization_msgs::Marker::DELETEALL;
        arr.markers.push_back(del);

        const auto& out = sim_.lastOutput();
        int id = 0;
        for (const auto& traj : out.local.rejected_candidates) {
            if (traj.points.empty()) continue;
            visualization_msgs::Marker mk;
            mk.header = header();
            mk.ns = "rejected";
            mk.id = id++;
            mk.type = visualization_msgs::Marker::LINE_STRIP;
            mk.action = visualization_msgs::Marker::ADD;
            mk.scale.x = 0.02;
            mk.color.r = 0.55f; mk.color.g = 0.55f; mk.color.b = 0.55f;
            mk.color.a = 0.35f;
            for (const auto& p : traj.points) {
                geometry_msgs::Point pt;
                pt.x = p.x(); pt.y = p.y(); pt.z = 0.05;
                mk.points.push_back(pt);
            }
            arr.markers.push_back(std::move(mk));
        }
        pub_rejected_.publish(arr);
    }

    void publishMarkers() {
        const SimSnapshot& s = sim_.snapshot();
        const auto& out = sim_.lastOutput();
        const auto& scene = sim_.scene();
        const auto& veh = sim_.vehicle().state();
        visualization_msgs::MarkerArray arr;
        int id = 0;

        // DELETEALL first: when objects disappear (blocker, local target,
        // velocity, labels) the old markers must not linger in RViz.
        visualization_msgs::Marker del;
        del.header = header();
        del.ns = "debug";
        del.id = 0;
        del.action = visualization_msgs::Marker::DELETEALL;
        arr.markers.push_back(del);

        auto text = [&](const std::string& ns, double x, double y,
                        const std::string& txt, float r, float g, float b,
                        double scale) {
            visualization_msgs::Marker mk;
            mk.header = header();
            mk.ns = ns;
            mk.id = id++;
            mk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            mk.action = visualization_msgs::Marker::ADD;
            mk.pose.position.x = x; mk.pose.position.y = y; mk.pose.position.z = 0.1;
            mk.pose.orientation.w = 1.0;
            mk.scale.z = scale;
            mk.color.r = r; mk.color.g = g; mk.color.b = b; mk.color.a = 1.0;
            mk.text = txt;
            arr.markers.push_back(mk);
        };

        // Vehicle triangle.
        visualization_msgs::Marker tri;
        tri.header = header();
        tri.ns = "vehicle";
        tri.id = id++;
        tri.type = visualization_msgs::Marker::TRIANGLE_LIST;
        tri.action = visualization_msgs::Marker::ADD;
        tri.scale.x = 1.0; tri.scale.y = 1.0; tri.scale.z = 1.0;
        tri.color.r = 1.0f; tri.color.g = 0.9f; tri.color.b = 0.2f; tri.color.a = 1.0f;
        const double yaw = veh.yaw;
        const double L = 0.45, W2 = 0.28;
        const Vec2d c = veh.position;
        auto vp = [&](double lx, double ly) {
            const Vec2d v = c + rot2(Vec2d(lx, ly), yaw);
            geometry_msgs::Point p;
            p.x = v.x(); p.y = v.y(); p.z = 0.1;
            return p;
        };
        tri.points.push_back(vp(L, 0.0));
        tri.points.push_back(vp(-L * 0.6, W2));
        tri.points.push_back(vp(-L * 0.6, -W2));
        arr.markers.push_back(tri);

        // FOV fan — TRIANGLE_LIST (ROS has NO TRIANGLE_FAN marker): one
        // triangle (centre, arc_i, arc_{i+1}) per fan slice.
        visualization_msgs::Marker fan;
        fan.header = header();
        fan.ns = "fov";
        fan.id = id++;
        fan.type = visualization_msgs::Marker::TRIANGLE_LIST;
        fan.action = visualization_msgs::Marker::ADD;
        fan.scale.x = 1.0; fan.scale.y = 1.0; fan.scale.z = 1.0;
        fan.color.r = 1.0f; fan.color.g = 1.0f; fan.color.b = 1.0f; fan.color.a = 0.10f;
        {
            geometry_msgs::Point pc;
            pc.x = c.x(); pc.y = c.y(); pc.z = 0.02;
            const int n = 24;
            for (int i = 0; i < n; ++i) {
                const double a0 = yaw - deg2rad(p_.obs_fov_deg) / 2.0 +
                                  deg2rad(p_.obs_fov_deg) * (static_cast<double>(i) / n);
                const double a1 = yaw - deg2rad(p_.obs_fov_deg) / 2.0 +
                                  deg2rad(p_.obs_fov_deg) * (static_cast<double>(i + 1) / n);
                const Vec2d v0 = c + Vec2d(std::cos(a0), std::sin(a0)) * p_.obs_range_m;
                const Vec2d v1 = c + Vec2d(std::cos(a1), std::sin(a1)) * p_.obs_range_m;
                geometry_msgs::Point p0, p1;
                p0.x = v0.x(); p0.y = v0.y(); p0.z = 0.02;
                p1.x = v1.x(); p1.y = v1.y(); p1.z = 0.02;
                fan.points.push_back(pc);
                fan.points.push_back(p0);
                fan.points.push_back(p1);
            }
        }
        arr.markers.push_back(fan);

        // Local target.
        if (s.local_target_valid) {
            visualization_msgs::Marker tgt;
            tgt.header = header();
            tgt.ns = "local_target";
            tgt.id = id++;
            tgt.type = visualization_msgs::Marker::SPHERE;
            tgt.action = visualization_msgs::Marker::ADD;
            tgt.pose.position.x = s.local_target.x();
            tgt.pose.position.y = s.local_target.y();
            tgt.pose.position.z = 0.08;
            tgt.pose.orientation.w = 1.0;
            tgt.scale.x = 0.3; tgt.scale.y = 0.3; tgt.scale.z = 0.3;
            tgt.color.r = 0.0f; tgt.color.g = 1.0f; tgt.color.b = 1.0f; tgt.color.a = 1.0f;
            arr.markers.push_back(tgt);
        }

        // Blocker — shown whenever an association outcome exists (so the
        // GUI can inspect NO_MATCH / AMBIGUOUS_MATCH after TASK_INVALID).
        if (out.blocker_association != BlockerAssociation::NONE) {
            // Privileged geometry when matched; otherwise show the LOCAL
            // evidence cluster (observation-only) with the failure state.
            // In local-only mode never leak matched truth geometry through
            // the otherwise non-truth debug_markers topic.
            const bool privileged = show_truth_ && out.blocker.found;
            const Vec2d bc = privileged ? out.blocker.center
                                        : out.local_blocker.visible_centroid;
            const double br = privileged ? out.blocker.radius
                                         : out.local_blocker.visible_radius;
            visualization_msgs::Marker blk;
            blk.header = header();
            blk.ns = "blocker";
            blk.id = id++;
            blk.type = visualization_msgs::Marker::SPHERE;
            blk.action = visualization_msgs::Marker::ADD;
            blk.pose.position.x = bc.x();
            blk.pose.position.y = bc.y();
            blk.pose.position.z = 0.06;
            blk.pose.orientation.w = 1.0;
            blk.scale.x = 2.0 * br;
            blk.scale.y = 2.0 * br;
            blk.scale.z = 0.02;
            // MATCHED → orange; NO_MATCH/AMBIGUOUS → red (failure).
            const bool failed =
                out.blocker_association == BlockerAssociation::NO_MATCH ||
                out.blocker_association == BlockerAssociation::AMBIGUOUS_MATCH;
            blk.color.r = failed ? 1.0f : 1.0f;
            blk.color.g = failed ? 0.1f : 0.5f;
            blk.color.b = failed ? 0.1f : 0.0f;
            blk.color.a = 0.4f;
            arr.markers.push_back(blk);
            text("blocker_label", bc.x(), bc.y(),
                 std::string(privileged ? "BLOCKER #" : "EVIDENCE #") +
                     std::to_string(privileged && !out.blocker.obstacle_ids.empty()
                                        ? out.blocker.obstacle_ids.front()
                                        : out.local_blocker.cluster_id) + " " +
                     blockerAssociationName(out.blocker_association) +
                     (privileged ? "" : " (local evidence)"),
                 1.0f, failed ? 1.0f : 0.5f, failed ? 0.1f : 0.0f, 0.8);
        }

        // Velocity vector — ARROW with TWO explicit points:
        //   points[0] = vehicle position, points[1] = velocity tip.
        if (veh.velocity_world.norm() > 0.02) {
            visualization_msgs::Marker vel;
            vel.header = header();
            vel.ns = "velocity";
            vel.id = id++;
            vel.type = visualization_msgs::Marker::ARROW;
            vel.action = visualization_msgs::Marker::ADD;
            vel.pose.position.x = 0.0;  // unused when points are given
            vel.pose.position.y = 0.0;
            vel.pose.position.z = 0.12;
            vel.pose.orientation.w = 1.0;
            vel.scale.x = 0.05; vel.scale.y = 0.12; vel.scale.z = 0.12;
            vel.color.r = 1.0f; vel.color.g = 0.0f; vel.color.b = 1.0f; vel.color.a = 1.0f;
            const Vec2d tip = c + veh.velocity_world * 0.4;
            geometry_msgs::Point p0, p1;
            p0.x = c.x(); p0.y = c.y(); p0.z = 0.12;
            p1.x = tip.x(); p1.y = tip.y(); p1.z = 0.12;
            vel.points.push_back(p0);
            vel.points.push_back(p1);
            arr.markers.push_back(vel);
        }

        // FSM / macro text labels.
        text("fsm_label", c.x(), c.y() + 1.0,
             std::string(fsmStateName(s.fsm_state)) +
                 (s.macro_active ? " [MACRO]" : ""),
             1.0f, 1.0f, 1.0f, 1.2);
        if (s.macro_active) {
            text("macro_label", c.x(), c.y() - 1.2,
                 "side=" + std::string(sideName(s.side)) + " reason=" + s.side_reason,
                 1.0f, 0.8f, 0.2f, 1.0);
        }

        pub_markers_.publish(arr);
    }

    void publishMarkersObstacles() {
        // scene_obstacles (latched TRUTH): cylinders + start + goal +
        // region boundary, distinctly labelled PRIVILEGED/TRUTH.
        visualization_msgs::MarkerArray arr;
        int id = 0;
        // DELETEALL first: when obstacle count / start / goal change, the
        // old markers must not linger.
        visualization_msgs::Marker del;
        del.header = header();
        del.ns = "truth";
        del.id = 0;
        del.action = visualization_msgs::Marker::DELETEALL;
        arr.markers.push_back(del);

        const auto& scene = sim_.scene();
        const auto& task = sim_.task();
        for (const auto& o : scene.obstacles) {
            visualization_msgs::Marker mk;
            mk.header = header();
            mk.ns = "obstacle";
            mk.id = id++;
            mk.type = visualization_msgs::Marker::CYLINDER;
            mk.action = visualization_msgs::Marker::ADD;
            mk.pose.position.x = o.center.x();
            mk.pose.position.y = o.center.y();
            mk.pose.position.z = 0.0;
            mk.pose.orientation.w = 1.0;
            mk.scale.x = 2.0 * o.radius;
            mk.scale.y = 2.0 * o.radius;
            mk.scale.z = 0.3;
            mk.color.r = 0.55f; mk.color.g = 0.05f; mk.color.b = 0.05f; mk.color.a = 1.0f;
            arr.markers.push_back(mk);
        }
        if (task.valid) {
            visualization_msgs::Marker start;
            start.header = header();
            start.ns = "task";
            start.id = id++;
            start.type = visualization_msgs::Marker::SPHERE;
            start.action = visualization_msgs::Marker::ADD;
            start.pose.position.x = task.start.x();
            start.pose.position.y = task.start.y();
            start.pose.position.z = 0.05;
            start.pose.orientation.w = 1.0;
            start.scale.x = 0.35; start.scale.y = 0.35; start.scale.z = 0.35;
            start.color.r = 0.0f; start.color.g = 1.0f; start.color.b = 0.0f; start.color.a = 1.0f;
            arr.markers.push_back(start);

            visualization_msgs::Marker goal;
            goal.header = header();
            goal.ns = "task";
            goal.id = id++;
            goal.type = visualization_msgs::Marker::SPHERE;
            goal.action = visualization_msgs::Marker::ADD;
            goal.pose.position.x = task.goal.x();
            goal.pose.position.y = task.goal.y();
            goal.pose.position.z = 0.05;
            goal.pose.orientation.w = 1.0;
            goal.scale.x = 0.4; goal.scale.y = 0.4; goal.scale.z = 0.4;
            goal.color.r = 1.0f; goal.color.g = 0.8f; goal.color.b = 0.0f; goal.color.a = 1.0f;
            arr.markers.push_back(goal);
        }
        // PENDING (not yet accepted) final goal — translucent, distinct.
        if (sim_.hasPendingGoal()) {
            const Vec2d pg = sim_.pendingGoal();
            visualization_msgs::Marker pending;
            pending.header = header();
            pending.ns = "pending_goal";
            pending.id = id++;
            pending.type = visualization_msgs::Marker::SPHERE;
            pending.action = visualization_msgs::Marker::ADD;
            pending.pose.position.x = pg.x();
            pending.pose.position.y = pg.y();
            pending.pose.position.z = 0.05;
            pending.pose.orientation.w = 1.0;
            pending.scale.x = 0.45; pending.scale.y = 0.45; pending.scale.z = 0.45;
            pending.color.r = 1.0f; pending.color.g = 0.6f; pending.color.b = 0.0f;
            pending.color.a = 0.45f;
            arr.markers.push_back(pending);

            visualization_msgs::Marker plabel;
            plabel.header = header();
            plabel.ns = "pending_goal";
            plabel.id = id++;
            plabel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            plabel.action = visualization_msgs::Marker::ADD;
            plabel.pose.position.x = pg.x();
            plabel.pose.position.y = pg.y() - 0.6;
            plabel.pose.position.z = 0.2;
            plabel.pose.orientation.w = 1.0;
            plabel.scale.z = 0.8;
            plabel.color.r = 1.0f; plabel.color.g = 0.6f; plabel.color.b = 0.0f;
            plabel.color.a = 1.0f;
            plabel.text = "PENDING GOAL";
            arr.markers.push_back(plabel);
        }
        // Region boundary (LINE_STRIP).
        visualization_msgs::Marker bounds;
        bounds.header = header();
        bounds.ns = "region";
        bounds.id = id++;
        bounds.type = visualization_msgs::Marker::LINE_STRIP;
        bounds.action = visualization_msgs::Marker::ADD;
        bounds.scale.x = 0.03;
        bounds.color.r = 1.0f; bounds.color.g = 1.0f; bounds.color.b = 1.0f; bounds.color.a = 0.6f;
        const double x0 = scene.min_bounds.x(), x1 = scene.max_bounds.x();
        const double y0 = scene.min_bounds.y(), y1 = scene.max_bounds.y();
        for (const auto& v : std::vector<Vec2d>{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}, {x0, y0}}) {
            geometry_msgs::Point p;
            p.x = v.x(); p.y = v.y(); p.z = 0.01;
            bounds.points.push_back(p);
        }
        arr.markers.push_back(bounds);

        visualization_msgs::Marker label;
        label.header = header();
        label.ns = "truth_label";
        label.id = id++;
        label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::Marker::ADD;
        label.pose.position.x = 0.5 * (x0 + x1);
        label.pose.position.y = y1 + 0.8;
        label.pose.position.z = 0.2;
        label.pose.orientation.w = 1.0;
        label.scale.z = 1.0;
        label.color.r = 1.0f; label.color.g = 0.3f; label.color.b = 0.3f; label.color.a = 1.0f;
        label.text = "PRIVILEGED / TRUTH";
        arr.markers.push_back(label);

        pub_obstacles_.publish(arr);
    }

    static std::vector<Vec2d> trajToPts(const Trajectory2D& t) {
        return t.points;
    }

    Params2D p_;
    ros::NodeHandle nh_, pnh_;
    DebugSimulation sim_;
    std::mutex mtx_;
    ros::Timer timer_;
    bool paused_ = true;
    bool show_truth_ = true;
    double sim_speed_ = 1.0;
    uint64_t seed_param_ = 42;
    uint64_t last_scene_id_ = std::numeric_limits<uint64_t>::max();
    uint64_t last_task_id_ = std::numeric_limits<uint64_t>::max();
    uint64_t last_goal_revision_ = std::numeric_limits<uint64_t>::max();
    bool last_pending_ = false;
    uint64_t last_pending_revision_ = std::numeric_limits<uint64_t>::max();

    ros::Publisher pub_snapshot_, pub_obs_, pub_patch_, pub_esdf_, pub_selectable_,
        pub_local_plan_, pub_executed_, pub_left_, pub_right_, pub_locked_,
        pub_rejected_, pub_markers_, pub_obstacles_;
    ros::ServiceServer srv_pause_, srv_step_, srv_reset_, srv_new_scene_,
        srv_new_task_, srv_goal_, srv_speed_, srv_export_log_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "debug_simulation_node");
    ros::NodeHandle nh, pnh("~");
    const Params2D p = loadParams(pnh);
    SimNode node(p, nh, pnh);
    ros::spin();
    return 0;
}
