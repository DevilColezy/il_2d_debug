#pragma once
/// @file   macro_expert_5hz.hpp
/// @brief  5 Hz privileged macro expert.
///
/// Responsibilities (all gated by the FSM, invoked exactly on the 5 Hz
/// boundary, i.e. every 6th 30 Hz tick):
///   1. identify the blocker (observed OCCUPIED cluster blocking the
///      swept corridor to the goal);
///   2. select LEFT or RIGHT strictly from CURRENT LOCAL OBSERVATION
///      "visible prefix evidence" (never from global route cost);
///   3. build / refresh the full safe route for the SELECTED side using
///      the global truth ESDF (privileged — allowed here);
///   4. emit a local target at a lookahead distance along the locked
///      route (the ONLY thing the 30 Hz planner receives);
///   5. evaluate the macro-exit condition set.
///
/// Side-selection causality rule:
///   * if the current FOV clearly favours one side → choose it, even if
///     global truth knows that side's route is longer;
///   * if the FOV cannot distinguish → AMBIGUOUS_DEFAULT_RIGHT;
///   * the chosen side is LOCKED until the blocker is passed; the FSM
///     marks the task TASK_INVALID_FOR_CAUSAL_RULE if the chosen side is
///     globally infeasible (never silently switch sides).

#include "il_2d_multiscale_debug/left_right_route_planner.hpp"
#include "il_2d_multiscale_debug/types.hpp"

namespace il_2d_multiscale_debug {

struct PreviewResult;

class MacroExpert5Hz {
public:
    explicit MacroExpert5Hz(const Params2D& p) : p_(p) {}

    /// Visible-prefix side evidence from the CURRENT INSTANTANEOUS FOV
    /// patch (never the merged history map).  `obs` MUST be the raw
    /// current_patch of this tick.  Scores are the AVERAGE visible free
    /// range actually used for the comparison.
    SideEvidence selectSideFromVisibleEvidence(const VehicleState2D& state,
                                               const LocalObservation& obs,
                                               const Vec2d& goal) const;

    /// Identify the LOCAL blocker EVIDENCE — the visible OCCUPIED cluster
    /// inside the swept corridor to the goal.  Uses ONLY the CURRENT
    /// INSTANTANEOUS FOV patch (`obs`); it carries no truth geometry and
    /// stores the OCCUPIED cell world coordinates (no truth ids).
    /// Cluster selection prefers the cluster that genuinely intersects the
    /// blocked planning corridor and is nearest in FORWARD distance (not
    /// nearest in Euclidean distance).
    LocalBlockerEvidence identifyBlocker(const VehicleState2D& state,
                                         const Vec2d& goal,
                                         const LocalObservation& obs) const;

    /// PRIVILEGED blocker geometry: deterministically associate the local
    /// cluster's visible cells with the true cylinders via per-cell
    /// surface/inside support.  Returns MATCHED (one dominant cylinder —
    /// no merging of passable-neighbour cylinders), NO_MATCH or
    /// AMBIGUOUS_MATCH.  Routes / gateways / homotopy use the matched
    /// cylinder geometry — never the visible surface cluster.
    BlockerInfo resolvePrivilegedBlocker(const LocalBlockerEvidence& ev,
                                         const Scene2D& scene) const;

    /// Build/refresh both routes (privileged).  `ref` is the FIXED
    /// homotopy reference captured at macro entry (gateways / side bias /
    /// homotopy are interpreted only relative to it).  Returns the one
    /// matching `side` as `locked`.
    void buildRoutes(const Scene2D& scene, const TruthEsdf2D& esdf,
                     const VehicleState2D& state, const Vec2d& goal,
                     const BlockerInfo& blocker, SideSelection side,
                     const HomotopyReference* ref, Route2D& left,
                     Route2D& right, Route2D& locked);

    /// Plain global-ESDF guidance route (no gateways / no homotopy) used
    /// once the blocker is passed (blocker_passed_latched).  Never fails
    /// the task; an invalid route makes the caller fall back to the goal.
    Route2D buildPlainRoute(const TruthEsdf2D& esdf, const VehicleState2D& state,
                            const Vec2d& goal) const;

    /// Rolling macro guide point (carrot) at an ADAPTIVE arc-length
    /// lookahead along the locked route.  The guide is selected from the
    /// vehicle's continuous arc-length projection onto the route, advanced
    /// monotonically (world-space hysteresis + a hard minimum distance so a
    /// foot-level 0.2-0.3 m target is never handed to the 30 Hz planner),
    /// clamped into the local FOV, and clipped to the farthest sufficiently
    /// distant point whose straight chord satisfies the macro/local handoff
    /// clearance. If no such long chord exists, the directional route carrot
    /// is retained and the 30 Hz layer still performs its normal local safety
    /// validation. `lookahead_used` / `route_progress` / `update_reason` are
    /// diagnostics. `force_advance` projects the previous carrot onto the refreshed route
    /// and moves beyond it; it is used by the FSM after sustained no-progress
    /// instead of merely clearing hysteresis and recomputing the same point.
    /// They are filled for logging (CSV v7 / GUI). Increments `update_event`
    /// (audit) via the caller.  Falls back to the goal on a degenerate route.
    LocalTarget guidanceTarget(const VehicleState2D& state,
                               const Route2D& locked, const Vec2d& goal,
                               const TruthEsdf2D& esdf, uint64_t tick,
                               bool force_advance,
                               double& lookahead_used,
                               double& route_progress,
                               std::string& update_reason);

    /// Reset the rolling-guide bookkeeping (monotonic progress + hysteresis)
    /// on macro entry / goal change.  The FSM owns no-progress timing.
    void resetGuideState();

    /// Exit-condition check (evaluated at 5 Hz boundaries by the FSM).
    /// `blocker_passed` is computed by the FSM from the FIXED macro-entry
    /// locked-route progress — never from the changing goal direction.
    /// v5: `local_preview` must be a SAFE AND PROGRESSING trajectory (the
    /// PreviewResult carries has_progressing_trajectory / executable
    /// progress / safe prefix / output speed / failure reason); a safe stop
    /// alone never satisfies local_precheck_ok.
    MacroExitCheck checkExitConditions(const VehicleState2D& state,
                                       const Vec2d& goal,
                                       const PreviewResult& local_preview,
                                       bool blocker_passed) const;

private:
    /// Free range along `bearing` (rad, relative to vehicle yaw) until the
    /// first non-FREE cell in the local observation.
    double freeRangeAlong(const VehicleState2D& state,
                          const LocalObservation& obs, double bearing) const;

    /// Interpolate the route point at arc length `arc` (clamped to
    /// [0, total]).
    Vec2d pointAtArc(const std::vector<Vec2d>& w,
                     const std::vector<double>& seg_start,
                     const std::vector<double>& seg_len, double arc,
                     double total) const;
    /// Continuous arc-length projection of `pos` onto the route: returns
    /// the vehicle progress (m) and lateral distance (m).
    bool projectOnRoute(const std::vector<Vec2d>& w,
                        const std::vector<double>& seg_start,
                        const std::vector<double>& seg_len, const Vec2d& pos,
                        double& progress, double& lateral) const;
    /// Smooth [0.5, 1] factor shrinking the lookahead on curved sections:
    /// 1 on straight stretches, ~0.5 at high curvature (mean turn angle
    /// over a window around `arc`).
    double routeCurvatureFactor(const std::vector<Vec2d>& w,
                                const std::vector<double>& seg_start,
                                const std::vector<double>& seg_len,
                                double arc) const;

    Params2D p_;
    // Rolling-guide bookkeeping (v7): last delivered guide world position
    // and whether it is valid (used for world-space hysteresis / continuity).
    Vec2d last_guide_pos_{0.0, 0.0};
    bool last_guide_valid_ = false;
};

}  // namespace il_2d_multiscale_debug
