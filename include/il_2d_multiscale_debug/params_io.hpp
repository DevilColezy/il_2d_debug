#pragma once
/// @file   params_io.hpp
/// @brief  Shared ROS-param → Params2D loader used by both nodes.
///         Key names must match config/default.yaml exactly.

#include "il_2d_multiscale_debug/types.hpp"

#include <ros/ros.h>
#include <XmlRpcValue.h>

#include <cstdint>
#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <vector>

namespace il_2d_multiscale_debug {

/// Read a uint64_t ROS param WITHOUT relying on XmlRpc's int type.
///
/// Supports:
///   * string values (seeds larger than 2^31 must be passed as strings,
///     e.g. <param name="seed" value="$(arg seed)" type="str"/>);
///   * non-negative int values;
///   * conversion-failure and overflow detection.
///
/// Returns false when the param is absent, is not a string/int, is
/// negative, fails to parse, or overflows.  Seed 0 is a VALID value —
/// callers that need "use the default" must add an explicit
/// `use_default_seed` flag instead of interpreting 0 as "default".
inline bool readUint64Param(const ros::NodeHandle& nh, const std::string& key,
                            uint64_t& out) {
    XmlRpc::XmlRpcValue v;
    if (!nh.getParam(key, v)) return false;
    if (v.getType() == XmlRpc::XmlRpcValue::TypeString) {
        const std::string s = static_cast<std::string>(v);
        if (s.empty()) return false;
        // Reject leading '-'.
        if (s[0] == '-') return false;
        try {
            size_t pos = 0;
            unsigned long long val = std::stoull(s, &pos, 10);
            if (pos != s.size()) return false;  // trailing garbage
            out = static_cast<uint64_t>(val);
            return true;
        } catch (const std::exception&) {
            return false;  // std::out_of_range / std::invalid_argument
        }
    }
    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        const int i = static_cast<int>(v);
        if (i < 0) return false;
        out = static_cast<uint64_t>(i);
        return true;
    }
    return false;
}

/// Read a uint32_t ROS param safely (XmlRpc has no uint32 type).
/// Accepts a non-negative XmlRpc int or a decimal string; rejects
/// negative values, parse failures and values above uint32 max.
/// 0 is a VALID value.  Returns false when the param is absent/invalid
/// so the caller can keep the Params2D default (with a ROS_WARN).
inline bool readUint32Param(const ros::NodeHandle& nh, const std::string& key,
                            uint32_t& out) {
    XmlRpc::XmlRpcValue v;
    if (!nh.getParam(key, v)) return false;
    if (v.getType() == XmlRpc::XmlRpcValue::TypeString) {
        const std::string s = static_cast<std::string>(v);
        if (s.empty() || s[0] == '-') return false;
        try {
            size_t pos = 0;
            unsigned long long val = std::stoull(s, &pos, 10);
            if (pos != s.size()) return false;
            if (val > std::numeric_limits<uint32_t>::max()) return false;
            out = static_cast<uint32_t>(val);
            return true;
        } catch (const std::exception&) {
            return false;
        }
    }
    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        const int i = static_cast<int>(v);
        if (i < 0) return false;
        out = static_cast<uint32_t>(i);
        return true;
    }
    return false;
}

inline std::vector<double> readDoubleList(const ros::NodeHandle& nh,
                                          const std::string& key,
                                          const std::vector<double>& def) {
    XmlRpc::XmlRpcValue v;
    if (nh.getParam(key, v) && v.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        std::vector<double> out;
        out.reserve(static_cast<size_t>(v.size()));
        for (int i = 0; i < v.size(); ++i) {
            out.push_back(static_cast<double>(v[i]));
        }
        return out;
    }
    return def;
}

inline Params2D loadParams(const ros::NodeHandle& pnh) {
    Params2D p;
    pnh.param("region/min_x", p.region_min_x, p.region_min_x);
    pnh.param("region/max_x", p.region_max_x, p.region_max_x);
    pnh.param("region/min_y", p.region_min_y, p.region_min_y);
    pnh.param("region/max_y", p.region_max_y, p.region_max_y);
    pnh.param("esdf_resolution", p.esdf_resolution, p.esdf_resolution);
    pnh.param("drone_radius", p.drone_radius, p.drone_radius);

    pnh.param("scene_generation/min_obstacles", p.scene_min_obstacles, p.scene_min_obstacles);
    pnh.param("scene_generation/max_obstacles", p.scene_max_obstacles, p.scene_max_obstacles);
    pnh.param("scene_generation/min_radius", p.scene_min_radius, p.scene_min_radius);
    pnh.param("scene_generation/max_radius", p.scene_max_radius, p.scene_max_radius);
    pnh.param("scene_generation/radius_distribution", p.scene_radius_distribution, p.scene_radius_distribution);
    pnh.param("scene_generation/safety_clearance", p.scene_safety_clearance, p.scene_safety_clearance);
    pnh.param("scene_generation/passage_margin", p.scene_passage_margin, p.scene_passage_margin);
    pnh.param("scene_generation/boundary_margin", p.scene_boundary_margin, p.scene_boundary_margin);
    pnh.param("scene_generation/max_attempts_per_obstacle", p.scene_max_attempts_per_obstacle, p.scene_max_attempts_per_obstacle);
    pnh.param("scene_generation/max_total_scene_attempts", p.scene_max_total_scene_attempts, p.scene_max_total_scene_attempts);

    pnh.param("connectivity/neighbor", p.conn_neighbor, p.conn_neighbor);
    pnh.param("connectivity/min_main_component_area_m2", p.conn_min_main_component_area_m2, p.conn_min_main_component_area_m2);

    pnh.param("task_sampling/min_start_goal_distance", p.task_min_start_goal_distance, p.task_min_start_goal_distance);
    pnh.param("task_sampling/yaw_bias_min_deg", p.task_yaw_bias_min_deg, p.task_yaw_bias_min_deg);
    pnh.param("task_sampling/yaw_bias_max_deg", p.task_yaw_bias_max_deg, p.task_yaw_bias_max_deg);
    pnh.param("task_sampling/goal_tolerance", p.task_goal_tolerance, p.task_goal_tolerance);
    pnh.param("task_sampling/episode_timeout_s", p.task_episode_timeout_s, p.task_episode_timeout_s);
    pnh.param("task_sampling/astar_confirm", p.task_astar_confirm, p.task_astar_confirm);
    pnh.param("task_sampling/goal_snap_max_radius_cells", p.task_goal_snap_max_radius_cells, p.task_goal_snap_max_radius_cells);
    pnh.param("task_sampling/max_sampling_attempts", p.task_max_sampling_attempts, p.task_max_sampling_attempts);
    pnh.param("task_sampling/require_both_sides_feasible", p.task_require_both_sides_feasible, p.task_require_both_sides_feasible);
    pnh.param("task_sampling/max_blocked_qual_attempts", p.task_max_blocked_qual_attempts, p.task_max_blocked_qual_attempts);

    pnh.param("observation/fov_deg", p.obs_fov_deg, p.obs_fov_deg);
    pnh.param("observation/range_m", p.obs_range_m, p.obs_range_m);
    pnh.param("observation/resolution", p.obs_resolution, p.obs_resolution);
    pnh.param("observation/ray_angular_res_deg", p.obs_ray_angular_res_deg, p.obs_ray_angular_res_deg);
    {
        uint32_t hist = p.obs_history_max_age_ticks;
        if (!readUint32Param(pnh, "observation/history_max_age_ticks", hist)) {
            ROS_WARN_STREAM("observation/history_max_age_ticks missing/invalid; "
                            << "keeping default " << p.obs_history_max_age_ticks);
        } else {
            p.obs_history_max_age_ticks = hist;
        }
    }

    pnh.param("local_planner/horizon_s", p.lp_horizon_s, p.lp_horizon_s);
    pnh.param("local_planner/dt", p.lp_dt, p.lp_dt);
    p.lp_speed_samples = readDoubleList(pnh, "local_planner/speed_samples", p.lp_speed_samples);
    p.lp_lateral_ratio_samples = readDoubleList(pnh, "local_planner/lateral_ratio_samples", p.lp_lateral_ratio_samples);
    p.lp_yaw_rate_samples = readDoubleList(pnh, "local_planner/yaw_rate_samples", p.lp_yaw_rate_samples);
    pnh.param("local_planner/max_speed", p.lp_max_speed, p.lp_max_speed);
    pnh.param("local_planner/max_accel", p.lp_max_accel, p.lp_max_accel);
    pnh.param("local_planner/max_yaw_rate", p.lp_max_yaw_rate, p.lp_max_yaw_rate);
    pnh.param("local_planner/max_yaw_accel", p.lp_max_yaw_accel, p.lp_max_yaw_accel);
    pnh.param("local_planner/min_clearance", p.lp_min_clearance, p.lp_min_clearance);
    pnh.param("local_planner/soft_clearance_radius_m", p.lp_soft_clearance_radius_m, p.lp_soft_clearance_radius_m);
    pnh.param("local_planner/clearance_discretization_margin_m", p.lp_clearance_discretization_margin_m, p.lp_clearance_discretization_margin_m);
    pnh.param("local_planner/obstacle_reaction_time_s", p.lp_obstacle_reaction_time_s, p.lp_obstacle_reaction_time_s);
    pnh.param("local_planner/control_period_s", p.lp_control_period_s, p.lp_control_period_s);
    pnh.param("local_planner/max_allowed_regress_m", p.lp_max_allowed_regress_m, p.lp_max_allowed_regress_m);
    pnh.param("local_planner/limit_cycle_window_ticks", p.lp_limit_cycle_window_ticks, p.lp_limit_cycle_window_ticks);
    pnh.param("local_planner/limit_cycle_net_progress_m", p.lp_limit_cycle_net_progress_m, p.lp_limit_cycle_net_progress_m);
    pnh.param("local_planner/limit_cycle_min_blocked_ticks", p.lp_limit_cycle_min_blocked_ticks, p.lp_limit_cycle_min_blocked_ticks);
    pnh.param("local_planner/limit_cycle_lateral_flip_count", p.lp_limit_cycle_lateral_flip_count, p.lp_limit_cycle_lateral_flip_count);
    pnh.param("local_planner/turn_enter_deg", p.lp_turn_enter_deg, p.lp_turn_enter_deg);
    pnh.param("local_planner/turn_exit_deg", p.lp_turn_exit_deg, p.lp_turn_exit_deg);
    pnh.param("local_planner/turn_exit_max_yaw_rate", p.lp_turn_exit_max_yaw_rate, p.lp_turn_exit_max_yaw_rate);
    pnh.param("local_planner/turn_k", p.lp_turn_k, p.lp_turn_k);
    pnh.param("local_planner/near_goal_heading_relax_distance", p.lp_near_goal_heading_relax_distance, p.lp_near_goal_heading_relax_distance);
    pnh.param("local_planner/near_goal_turn_enter_deg", p.lp_near_goal_turn_enter_deg, p.lp_near_goal_turn_enter_deg);
    pnh.param("local_planner/terminal_control_distance", p.lp_terminal_control_distance, p.lp_terminal_control_distance);
    pnh.param("local_planner/terminal_speed_gain", p.lp_terminal_speed_gain, p.lp_terminal_speed_gain);
    pnh.param("local_planner/terminal_max_speed", p.lp_terminal_max_speed, p.lp_terminal_max_speed);
    pnh.param("local_planner/terminal_max_yaw_rate", p.lp_terminal_max_yaw_rate, p.lp_terminal_max_yaw_rate);
    pnh.param("local_planner/min_progress_m", p.lp_min_progress_m, p.lp_min_progress_m);
    pnh.param("local_planner/min_progress_speed_mps", p.lp_min_progress_speed_mps, p.lp_min_progress_speed_mps);
    pnh.param("local_planner/min_progress_epsilon_m", p.lp_min_progress_epsilon_m, p.lp_min_progress_epsilon_m);
    pnh.param("local_planner/target_discontinuity_reset_m", p.lp_target_discontinuity_reset_m, p.lp_target_discontinuity_reset_m);
    pnh.param("local_planner/nominal_clearance_m", p.lp_nominal_clearance_m, p.lp_nominal_clearance_m);
    pnh.param("local_planner/risk_corridor_half_width", p.lp_risk_corridor_half_width, p.lp_risk_corridor_half_width);
    pnh.param("local_planner/risk_distance_horizon_m", p.lp_risk_distance_horizon_m, p.lp_risk_distance_horizon_m);
    pnh.param("local_planner/risk_ttc_horizon_s", p.lp_risk_ttc_horizon_s, p.lp_risk_ttc_horizon_s);
    pnh.param("local_planner/risk_trajectory_radius_m", p.lp_risk_trajectory_radius_m, p.lp_risk_trajectory_radius_m);
    pnh.param("local_planner/avoidance_active_threshold", p.lp_avoidance_active_threshold, p.lp_avoidance_active_threshold);
    pnh.param("local_planner/brake_stop_margin_m", p.lp_brake_stop_margin_m, p.lp_brake_stop_margin_m);
    pnh.param("local_planner/min_executable_prefix_s", p.lp_min_executable_prefix_s, p.lp_min_executable_prefix_s);
    pnh.param("local_planner/scoring_horizon_s", p.lp_scoring_horizon_s, p.lp_scoring_horizon_s);
    pnh.param("local_planner/cost_tie_tolerance", p.lp_cost_tie_tolerance, p.lp_cost_tie_tolerance);
    pnh.param("local_planner/cross_track_normalize_m", p.lp_cross_track_normalize_m, p.lp_cross_track_normalize_m);
    pnh.param("local_planner/cost_weights/progress", p.cost_w_progress, p.cost_w_progress);
    pnh.param("local_planner/cost_weights/clearance", p.cost_w_clearance, p.cost_w_clearance);
    pnh.param("local_planner/cost_weights/smoothness", p.cost_w_smoothness, p.cost_w_smoothness);
    pnh.param("local_planner/cost_weights/speed_change", p.cost_w_speed_change, p.cost_w_speed_change);
    pnh.param("local_planner/cost_weights/yaw_rate_change", p.cost_w_yaw_rate_change, p.cost_w_yaw_rate_change);
    pnh.param("local_planner/cost_weights/terminal_heading", p.cost_w_terminal_heading, p.cost_w_terminal_heading);
    pnh.param("local_planner/cost_weights/velocity_alignment", p.cost_w_velocity_alignment, p.cost_w_velocity_alignment);
    pnh.param("local_planner/cost_weights/cross_track", p.cost_w_cross_track, p.cost_w_cross_track);
    pnh.param("local_planner/cost_weights/obstacle_risk", p.cost_w_obstacle_risk, p.cost_w_obstacle_risk);

    pnh.param("macro/local_failure_duration_s", p.macro_local_failure_duration_s, p.macro_local_failure_duration_s);
    pnh.param("macro/route_lookahead_min", p.macro_route_lookahead_min, p.macro_route_lookahead_min);
    pnh.param("macro/route_lookahead_max", p.macro_route_lookahead_max, p.macro_route_lookahead_max);
    pnh.param("macro/guide_min_distance_m", p.macro_guide_min_distance_m, p.macro_guide_min_distance_m);
    pnh.param("macro/guide_lookahead_time_gap_s", p.macro_guide_lookahead_time_gap_s, p.macro_guide_lookahead_time_gap_s);
    pnh.param("macro/guide_hysteresis_m", p.macro_guide_hysteresis_m, p.macro_guide_hysteresis_m);
    pnh.param("macro/guide_fov_margin_deg", p.macro_guide_fov_margin_deg, p.macro_guide_fov_margin_deg);
    pnh.param("macro/no_progress_threshold_m", p.macro_no_progress_threshold_m, p.macro_no_progress_threshold_m);
    pnh.param("macro/no_progress_duration_threshold_s", p.macro_no_progress_duration_threshold_s, p.macro_no_progress_duration_threshold_s);
    pnh.param("macro/side_evidence_margin", p.macro_side_evidence_margin, p.macro_side_evidence_margin);
    pnh.param("macro/evidence_ray_step_deg", p.macro_evidence_ray_step_deg, p.macro_evidence_ray_step_deg);
    pnh.param("macro/min_evidence_ray_pairs", p.macro_min_evidence_ray_pairs, p.macro_min_evidence_ray_pairs);
    pnh.param("macro/local_target_event_tolerance_m", p.macro_local_target_event_tolerance_m, p.macro_local_target_event_tolerance_m);
    pnh.param("macro/exit_fov_margin_deg", p.macro_exit_fov_margin_deg, p.macro_exit_fov_margin_deg);
    pnh.param("macro/corridor_half_width", p.macro_corridor_half_width, p.macro_corridor_half_width);
    pnh.param("macro/corridor_rear_tolerance_m", p.macro_corridor_rear_tolerance_m, p.macro_corridor_rear_tolerance_m);
    pnh.param("macro/blocking_lateral_span_ratio", p.macro_blocking_lateral_span_ratio, p.macro_blocking_lateral_span_ratio);
    pnh.param("macro/blocker_clear_dist_m", p.macro_blocker_clear_dist_m, p.macro_blocker_clear_dist_m);
    pnh.param("macro/blocker_projection_max_dist_m", p.macro_blocker_projection_max_dist_m, p.macro_blocker_projection_max_dist_m);
    pnh.param("macro/progress_forward_tolerance_m", p.macro_progress_forward_tolerance_m, p.macro_progress_forward_tolerance_m);
    pnh.param("macro/progress_back_window_m", p.macro_progress_back_window_m, p.macro_progress_back_window_m);
    pnh.param("macro/blocker_match_min_cells", p.macro_blocker_match_min_cells, p.macro_blocker_match_min_cells);
    pnh.param("macro/blocker_match_ambiguity_ratio", p.macro_blocker_match_ambiguity_ratio, p.macro_blocker_match_ambiguity_ratio);
    pnh.param("macro/blocker_match_surface_tol_m", p.macro_blocker_match_surface_tol_m, p.macro_blocker_match_surface_tol_m);
    pnh.param("macro/macro_exit_stable_ticks", p.macro_exit_stable_ticks, p.macro_exit_stable_ticks);
    pnh.param("macro/macro_reentry_guard_ticks", p.macro_reentry_guard_ticks, p.macro_reentry_guard_ticks);
    pnh.param("macro/route_clearance_margin", p.macro_route_clearance_margin, p.macro_route_clearance_margin);
    pnh.param("macro/start_recovery_max_radius_m", p.macro_start_recovery_max_radius_m, p.macro_start_recovery_max_radius_m);
    pnh.param("macro/unknown_recovery_threshold_ticks", p.macro_unknown_recovery_threshold_ticks, p.macro_unknown_recovery_threshold_ticks);
    pnh.param("macro/route_side_bias", p.macro_route_side_bias, p.macro_route_side_bias);
    pnh.param("macro/homotopy_side_tolerance_m", p.macro_homotopy_side_tolerance_m, p.macro_homotopy_side_tolerance_m);
    pnh.param("macro/gateway_projection_radius_m", p.macro_gateway_projection_radius_m, p.macro_gateway_projection_radius_m);

    pnh.param("vehicle/goal_stop_speed_mps", p.vehicle_goal_stop_speed_mps, p.vehicle_goal_stop_speed_mps);
    pnh.param("vehicle/stationary_speed_mps", p.vehicle_stationary_speed_mps, p.vehicle_stationary_speed_mps);

    pnh.param("gui/refresh_rate_hz", p.gui_refresh_rate_hz, p.gui_refresh_rate_hz);
    pnh.param("gui/default_speed", p.gui_default_speed, p.gui_default_speed);
    p.gui_speeds = readDoubleList(pnh, "gui/speeds", p.gui_speeds);
    pnh.param("gui/show_esdf", p.gui_show_esdf, p.gui_show_esdf);
    pnh.param("gui/show_truth_paths", p.gui_show_truth_paths, p.gui_show_truth_paths);
    pnh.param("gui/show_local_observation", p.gui_show_local_observation, p.gui_show_local_observation);
    pnh.param("gui/show_rejected_candidates", p.gui_show_rejected_candidates, p.gui_show_rejected_candidates);

    // ── Logging (flight-log export directory + startup cleanup) ──────
    pnh.param("logging/output_directory", p.logging_output_directory, p.logging_output_directory);
    pnh.param("logging/clear_on_start", p.logging_clear_on_start, p.logging_clear_on_start);
    pnh.param("logging/filename_prefix", p.logging_filename_prefix, p.logging_filename_prefix);

    {
        uint64_t dseed = p.default_seed;
        if (!readUint64Param(pnh, "default_seed", dseed)) {
            ROS_WARN_STREAM("default_seed missing/invalid; keeping default "
                            << p.default_seed);
        } else {
            p.default_seed = dseed;
        }
    }

    // Keep the new recovery/prefix parameters inside their documented
    // semantic domain.  Both nodes call this same loader, so planning and
    // GUI diagnostics always agree even when a user supplies bad YAML.
    if (!(p.lp_dt > 0.0)) {
        ROS_WARN_STREAM("local_planner/dt must be > 0; using 0.1");
        p.lp_dt = 0.1;
    }
    if (!std::isfinite(p.lp_horizon_s) || p.lp_horizon_s < p.lp_dt) {
        ROS_WARN_STREAM("local_planner/horizon_s must be >= dt; clamping to "
                        << p.lp_dt);
        p.lp_horizon_s = p.lp_dt;
    }
    if (!std::isfinite(p.lp_min_executable_prefix_s) ||
        p.lp_min_executable_prefix_s < p.lp_dt) {
        ROS_WARN_STREAM("local_planner/min_executable_prefix_s must be >= dt; "
                        "clamping to " << p.lp_dt);
        p.lp_min_executable_prefix_s = p.lp_dt;
    } else if (p.lp_min_executable_prefix_s > p.lp_horizon_s) {
        ROS_WARN_STREAM("local_planner/min_executable_prefix_s exceeds horizon; "
                        "clamping to " << p.lp_horizon_s);
        p.lp_min_executable_prefix_s = p.lp_horizon_s;
    }
    if (!std::isfinite(p.lp_scoring_horizon_s)) {
        ROS_WARN_STREAM("local_planner/scoring_horizon_s must be finite; "
                        "using 0.8");
        p.lp_scoring_horizon_s = 0.8;
    }
    p.lp_scoring_horizon_s =
        clamp(p.lp_scoring_horizon_s, p.lp_min_executable_prefix_s,
              p.lp_horizon_s);
    if (p.macro_unknown_recovery_threshold_ticks < 1) {
        ROS_WARN_STREAM("macro/unknown_recovery_threshold_ticks must be >= 1; "
                        "clamping to 1");
        p.macro_unknown_recovery_threshold_ticks = 1;
    }
    if (!std::isfinite(p.lp_turn_exit_max_yaw_rate) ||
        p.lp_turn_exit_max_yaw_rate < 0.0) {
        ROS_WARN_STREAM("local_planner/turn_exit_max_yaw_rate must be finite "
                        "and >= 0; using 0.15");
        p.lp_turn_exit_max_yaw_rate = 0.15;
    } else if (std::isfinite(p.lp_max_yaw_rate) && p.lp_max_yaw_rate > 0.0 &&
               p.lp_turn_exit_max_yaw_rate > p.lp_max_yaw_rate) {
        ROS_WARN_STREAM("local_planner/turn_exit_max_yaw_rate exceeds "
                        "max_yaw_rate; clamping to " << p.lp_max_yaw_rate);
        p.lp_turn_exit_max_yaw_rate = p.lp_max_yaw_rate;
    }
    if (!std::isfinite(p.lp_turn_exit_deg) || p.lp_turn_exit_deg < 0.0 ||
        p.lp_turn_exit_deg >= 179.0) {
        ROS_WARN_STREAM("local_planner/turn_exit_deg must be in [0,179); "
                        "using 8.0");
        p.lp_turn_exit_deg = 8.0;
    }
    if (!std::isfinite(p.lp_turn_enter_deg) ||
        p.lp_turn_enter_deg <= p.lp_turn_exit_deg ||
        p.lp_turn_enter_deg >= 180.0) {
        ROS_WARN_STREAM("local_planner/turn_enter_deg must be greater than "
                        "turn_exit_deg and < 180; using 42.0");
        p.lp_turn_enter_deg = std::max(42.0, p.lp_turn_exit_deg + 1.0);
    }
    if (!std::isfinite(p.lp_near_goal_heading_relax_distance) ||
        p.lp_near_goal_heading_relax_distance < 0.0) {
        ROS_WARN_STREAM("local_planner/near_goal_heading_relax_distance must "
                        "be finite and >= 0; using 1.0");
        p.lp_near_goal_heading_relax_distance = 1.0;
    }
    if (!std::isfinite(p.lp_near_goal_turn_enter_deg) ||
        p.lp_near_goal_turn_enter_deg < p.lp_turn_enter_deg ||
        p.lp_near_goal_turn_enter_deg >= 180.0) {
        ROS_WARN_STREAM("local_planner/near_goal_turn_enter_deg must be "
                        ">= turn_enter_deg and < 180; using 75.0");
        p.lp_near_goal_turn_enter_deg =
            std::max(75.0, p.lp_turn_enter_deg);
    }
    if (!std::isfinite(p.lp_terminal_control_distance) ||
        p.lp_terminal_control_distance < p.task_goal_tolerance) {
        ROS_WARN_STREAM("local_planner/terminal_control_distance must be "
                        "finite and >= task goal_tolerance; using 1.2");
        p.lp_terminal_control_distance =
            std::max(1.2, p.task_goal_tolerance);
    }
    if (!std::isfinite(p.lp_terminal_speed_gain) ||
        p.lp_terminal_speed_gain <= 0.0) {
        ROS_WARN_STREAM("local_planner/terminal_speed_gain must be finite "
                        "and > 0; using 1.0");
        p.lp_terminal_speed_gain = 1.0;
    }
    if (!std::isfinite(p.lp_terminal_max_speed) ||
        p.lp_terminal_max_speed <= 0.0) {
        ROS_WARN_STREAM("local_planner/terminal_max_speed must be finite "
                        "and > 0; using 0.6");
        p.lp_terminal_max_speed = 0.6;
    }
    p.lp_terminal_max_speed =
        std::min(p.lp_terminal_max_speed, p.lp_max_speed);
    if (!std::isfinite(p.lp_terminal_max_yaw_rate) ||
        p.lp_terminal_max_yaw_rate < 0.0) {
        ROS_WARN_STREAM("local_planner/terminal_max_yaw_rate must be finite "
                        "and >= 0; using 0.5");
        p.lp_terminal_max_yaw_rate = 0.5;
    }
    p.lp_terminal_max_yaw_rate =
        std::min(p.lp_terminal_max_yaw_rate, p.lp_max_yaw_rate);
    if (!std::isfinite(p.lp_cost_tie_tolerance) ||
        p.lp_cost_tie_tolerance < 0.0) {
        ROS_WARN_STREAM("local_planner/cost_tie_tolerance must be finite and "
                        ">= 0; using 1e-6");
        p.lp_cost_tie_tolerance = 1e-6;
    }
    if (!std::isfinite(p.lp_cross_track_normalize_m) ||
        p.lp_cross_track_normalize_m <= 0.0) {
        ROS_WARN_STREAM("local_planner/cross_track_normalize_m must be finite "
                        "and > 0; using 2.0");
        p.lp_cross_track_normalize_m = 2.0;
    }
    auto nonnegativeWeight = [](double& value, const char* key,
                                double fallback) {
        if (!std::isfinite(value) || value < 0.0) {
            ROS_WARN_STREAM(key << " must be finite and >= 0; using "
                                << fallback);
            value = fallback;
        }
    };
    nonnegativeWeight(p.cost_w_progress,
                      "local_planner/cost_weights/progress", 1.0);
    nonnegativeWeight(p.cost_w_clearance,
                      "local_planner/cost_weights/clearance", 2.0);
    nonnegativeWeight(p.cost_w_smoothness,
                      "local_planner/cost_weights/smoothness", 0.5);
    nonnegativeWeight(p.cost_w_speed_change,
                      "local_planner/cost_weights/speed_change", 0.3);
    nonnegativeWeight(p.cost_w_yaw_rate_change,
                      "local_planner/cost_weights/yaw_rate_change", 0.3);
    nonnegativeWeight(p.cost_w_terminal_heading,
                      "local_planner/cost_weights/terminal_heading", 1.5);
    nonnegativeWeight(p.cost_w_velocity_alignment,
                      "local_planner/cost_weights/velocity_alignment", 1.2);
    nonnegativeWeight(p.cost_w_cross_track,
                      "local_planner/cost_weights/cross_track", 1.0);
    nonnegativeWeight(p.cost_w_obstacle_risk,
                      "local_planner/cost_weights/obstacle_risk", 3.0);

    // ── v4 soft-clearance / dynamic-envelope parameter validation ──
    if (!std::isfinite(p.lp_soft_clearance_radius_m) ||
        p.lp_soft_clearance_radius_m <= p.lp_min_clearance) {
        ROS_WARN_STREAM("local_planner/soft_clearance_radius_m must be "
                        "finite and > min_clearance; using 2.0");
        p.lp_soft_clearance_radius_m =
            std::max(2.0, p.lp_min_clearance + 0.5);
    }
    if (!std::isfinite(p.lp_clearance_discretization_margin_m) ||
        p.lp_clearance_discretization_margin_m < 0.0) {
        ROS_WARN_STREAM("local_planner/clearance_discretization_margin_m "
                        "must be finite and >= 0; using 0.05");
        p.lp_clearance_discretization_margin_m = 0.05;
    }
    if (!std::isfinite(p.lp_obstacle_reaction_time_s) ||
        p.lp_obstacle_reaction_time_s < 0.0) {
        ROS_WARN_STREAM("local_planner/obstacle_reaction_time_s must be "
                        "finite and >= 0; using 0.20");
        p.lp_obstacle_reaction_time_s = 0.20;
    }
    if (!std::isfinite(p.lp_control_period_s) || p.lp_control_period_s <= 0.0) {
        ROS_WARN_STREAM("local_planner/control_period_s must be finite and "
                        "> 0; using 1/30");
        p.lp_control_period_s = 1.0 / 30.0;
    }
    if (!std::isfinite(p.lp_max_allowed_regress_m) ||
        p.lp_max_allowed_regress_m < 0.0) {
        ROS_WARN_STREAM("local_planner/max_allowed_regress_m must be finite "
                        "and >= 0; using 0.05");
        p.lp_max_allowed_regress_m = 0.05;
    }
    // ── v5 progress-qualification / discontinuity params ──────────
    if (!std::isfinite(p.lp_min_progress_speed_mps) ||
        p.lp_min_progress_speed_mps < 0.0) {
        ROS_WARN_STREAM("local_planner/min_progress_speed_mps must be finite "
                        "and >= 0; using 0.03");
        p.lp_min_progress_speed_mps = 0.03;
    }
    if (!std::isfinite(p.lp_min_progress_epsilon_m) ||
        p.lp_min_progress_epsilon_m < 0.0) {
        ROS_WARN_STREAM("local_planner/min_progress_epsilon_m must be finite "
                        "and >= 0; using 0.01");
        p.lp_min_progress_epsilon_m = 0.01;
    }
    if (!std::isfinite(p.lp_target_discontinuity_reset_m) ||
        p.lp_target_discontinuity_reset_m <= 0.0) {
        ROS_WARN_STREAM("local_planner/target_discontinuity_reset_m must be "
                        "finite and > 0; using 1.5");
        p.lp_target_discontinuity_reset_m = 1.5;
    }
    // ── v7 nominal clearance + early-avoidance risk params ─────────
    if (!std::isfinite(p.lp_nominal_clearance_m) ||
        p.lp_nominal_clearance_m <= p.lp_min_clearance) {
        ROS_WARN_STREAM("local_planner/nominal_clearance_m must be finite "
                        "and > min_clearance; using 0.65");
        p.lp_nominal_clearance_m =
            std::max(0.65, p.lp_min_clearance + 0.05);
    }
    if (!std::isfinite(p.lp_risk_corridor_half_width) ||
        p.lp_risk_corridor_half_width <= 0.0) {
        ROS_WARN_STREAM("local_planner/risk_corridor_half_width must be "
                        "finite and > 0; using 1.0");
        p.lp_risk_corridor_half_width = 1.0;
    }
    if (!std::isfinite(p.lp_risk_distance_horizon_m) ||
        p.lp_risk_distance_horizon_m <= p.lp_min_clearance) {
        ROS_WARN_STREAM("local_planner/risk_distance_horizon_m must be "
                        "finite and > min_clearance; using 5.0");
        p.lp_risk_distance_horizon_m = 5.0;
    }
    if (!std::isfinite(p.lp_risk_ttc_horizon_s) ||
        p.lp_risk_ttc_horizon_s <= 0.0) {
        ROS_WARN_STREAM("local_planner/risk_ttc_horizon_s must be finite "
                        "and > 0; using 2.5");
        p.lp_risk_ttc_horizon_s = 2.5;
    }
    if (!std::isfinite(p.lp_risk_trajectory_radius_m) ||
        p.lp_risk_trajectory_radius_m <= 0.0) {
        ROS_WARN_STREAM("local_planner/risk_trajectory_radius_m must be "
                        "finite and > 0; using 1.0");
        p.lp_risk_trajectory_radius_m = 1.0;
    }
    if (!std::isfinite(p.lp_avoidance_active_threshold) ||
        p.lp_avoidance_active_threshold < 0.0) {
        ROS_WARN_STREAM("local_planner/avoidance_active_threshold must be "
                        "finite and >= 0; using 0.10");
        p.lp_avoidance_active_threshold = 0.10;
    }
    if (p.lp_limit_cycle_window_ticks < 3) {
        ROS_WARN_STREAM("local_planner/limit_cycle_window_ticks must be "
                        ">= 3; clamping to 3");
        p.lp_limit_cycle_window_ticks = 3;
    }
    if (!std::isfinite(p.lp_limit_cycle_net_progress_m) ||
        p.lp_limit_cycle_net_progress_m < 0.0) {
        ROS_WARN_STREAM("local_planner/limit_cycle_net_progress_m must be "
                        "finite and >= 0; using 0.10");
        p.lp_limit_cycle_net_progress_m = 0.10;
    }
    if (p.lp_limit_cycle_min_blocked_ticks < 1) {
        ROS_WARN_STREAM("local_planner/limit_cycle_min_blocked_ticks must "
                        "be >= 1; clamping to 1");
        p.lp_limit_cycle_min_blocked_ticks = 1;
    }
    p.lp_limit_cycle_min_blocked_ticks = std::min(
        p.lp_limit_cycle_min_blocked_ticks, p.lp_limit_cycle_window_ticks);
    if (p.lp_limit_cycle_lateral_flip_count < 1) {
        ROS_WARN_STREAM("local_planner/limit_cycle_lateral_flip_count must "
                        "be >= 1; clamping to 1");
        p.lp_limit_cycle_lateral_flip_count = 1;
    }
    if (!std::isfinite(p.macro_start_recovery_max_radius_m) ||
        p.macro_start_recovery_max_radius_m < 0.0) {
        ROS_WARN_STREAM("macro/start_recovery_max_radius_m must be finite "
                        "and >= 0; using 0.5");
        p.macro_start_recovery_max_radius_m = 0.5;
    }
    // Keep the recovery search bounded (finite radius / finite node count).
    p.macro_start_recovery_max_radius_m =
        std::min(p.macro_start_recovery_max_radius_m, 5.0);
    // ── v7 rolling macro-guide params ──────────────────────────────
    if (!std::isfinite(p.macro_guide_min_distance_m) ||
        p.macro_guide_min_distance_m <= 0.0) {
        ROS_WARN_STREAM("macro/guide_min_distance_m must be finite and > 0; "
                        "using 1.5");
        p.macro_guide_min_distance_m = 1.5;
    }
    if (!std::isfinite(p.macro_guide_lookahead_time_gap_s) ||
        p.macro_guide_lookahead_time_gap_s < 0.0) {
        ROS_WARN_STREAM("macro/guide_lookahead_time_gap_s must be finite "
                        "and >= 0; using 1.0");
        p.macro_guide_lookahead_time_gap_s = 1.0;
    }
    if (!std::isfinite(p.macro_guide_hysteresis_m) ||
        p.macro_guide_hysteresis_m < 0.0) {
        ROS_WARN_STREAM("macro/guide_hysteresis_m must be finite and >= 0; "
                        "using 0.3");
        p.macro_guide_hysteresis_m = 0.3;
    }
    if (!std::isfinite(p.macro_guide_fov_margin_deg) ||
        p.macro_guide_fov_margin_deg < 0.0 ||
        p.macro_guide_fov_margin_deg >= 90.0) {
        ROS_WARN_STREAM("macro/guide_fov_margin_deg must be finite and in "
                        "[0,90); using 10.0");
        p.macro_guide_fov_margin_deg = 10.0;
    }
    if (!std::isfinite(p.macro_no_progress_threshold_m) ||
        p.macro_no_progress_threshold_m <= 0.0) {
        ROS_WARN_STREAM("macro/no_progress_threshold_m must be finite and "
                        "> 0; using 0.05");
        p.macro_no_progress_threshold_m = 0.05;
    }
    if (!std::isfinite(p.macro_no_progress_duration_threshold_s) ||
        p.macro_no_progress_duration_threshold_s <= 0.0) {
        ROS_WARN_STREAM("macro/no_progress_duration_threshold_s must be "
                        "finite and > 0; using 1.0");
        p.macro_no_progress_duration_threshold_s = 1.0;
    }
    if (p.logging_filename_prefix.empty() ||
        p.logging_filename_prefix == "." ||
        p.logging_filename_prefix == ".." ||
        p.logging_filename_prefix.find_first_of("/\\") != std::string::npos) {
        ROS_WARN_STREAM("logging/filename_prefix must be a non-empty filename "
                        "prefix without path separators; using flight_log_");
        p.logging_filename_prefix = "flight_log_";
    }
    return p;
}

}  // namespace il_2d_multiscale_debug
