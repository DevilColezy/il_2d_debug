#pragma once
/// @file   macro_expert_5hz.hpp
/// @brief  v9 5 Hz VisibilityTargetCorrector — local observability judge
///         + target corrector.
///
/// v9 REDESIGN: the 5 Hz expert is NO LONGER a macro detour planner.  It
/// answers exactly one question on every real 5 Hz boundary
/// (tick % 6 == 0):
///
///   "Do the current FOV and its causal local history contain enough
///    information for the 30 Hz expert to finish its OWN local avoidance?"
///
///   * YES outside a correction episode → PASS_THROUGH: keep the ORIGINAL
///     goal and do not touch the 30 Hz expert;
///   * YES during a correction episode → PASS_THROUGH only after the
///     ORIGINAL goal also returns to the safe ordinary-direction handoff
///     cone; until then, keep the locked side and issue NORMAL_CORRECTION;
///   * NO  → temporarily correct the tracked target:
///       - NORMAL_CORRECTION: a quantized in-FOV frontier on the locked
///         side (the 30 Hz expert moves toward it and keeps observing);
///       - TURN_LEFT / TURN_RIGHT: one bounded world-latched direction
///         step, initially just outside the FOV. Its live body bearing
///         converges while 30 Hz rotates; distance=1 forbids translation.
///
/// STRICT boundary rule:
///   "seeing an obstacle" ≠ "5 Hz takes over".
///   Only "seeing a blocker but NOT being able to see HOW to go around it
///   (occlusion / UNKNOWN / FOV truncation)" may trigger a correction.
///   When a local bypass first becomes visible, TURN changes to an ordinary
///   in-FOV NORMAL_CORRECTION on the locked side so the vehicle translates
///   around the blocker.  The correction is released only when local
///   avoidance remains observable AND the ORIGINAL goal is back inside the
///   ordinary-direction handoff cone AND actual yaw rate is below the shared
///   30 Hz turn-exit threshold.  A newly visible bypass first receives a
///   TURN->NORMAL bridge whenever a safe ordinary frontier is available.
///   This prevents TURN/PASS oscillation caused by residual yaw inertia.
///
/// INFORMATION BOUNDARY (enforced by the interface).  At runtime the
/// corrector may ONLY read:
///   * VehicleState2D (pose, velocity, yaw, yaw rate);
///   * the ORIGINAL final goal;
///   * the CURRENT INSTANTANEOUS FOV patch (current_patch);
///   * the causal, decaying local history map built only from past patches;
///   * the current tick;
///   * its own memory (correction_active, locked_side, current
///     directive, enter stability counter, re-entry guard, update
///     events, previous observability result).
/// It NEVER reads / receives / uses:
///   * PlannerResult / PreviewResult / FailureReason / PlannerStatus;
///   * previewPlan() / turn_mode / emergency_brake / blocked_observed;
///   * 30 Hz candidate trajectories / vx/vy/yaw_rate outputs;
///   * 30 Hz consecutive-failure counters / limit-cycle results;
///   * Scene2D / TruthEsdf2D / global ESDF / truth obstacle ids;
///   * global A* or global left/right routes.
///
/// The corrector NEVER plans a complete detour, NEVER keeps a rolling
/// guide, NEVER reads the 30 Hz outcome, and NEVER changes the 30 Hz
/// mission_revision (correction enter / refresh / exit only update the
/// directive update_event).

#include "il_2d_multiscale_debug/types.hpp"
#include "il_2d_multiscale_debug/effective_target_adapter.hpp"

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace il_2d_multiscale_debug {

class VisibilityTargetCorrector {
public:
    explicit VisibilityTargetCorrector(const Params2D& p)
        : p_(p), adapter_(p) {}

    /// Run on every real 5 Hz boundary (called exactly at tick % 6 == 0).
    /// Produces the zero-order-held TargetCorrectionDirective.  Pure local
    /// judgement; never reads any 30 Hz outcome.
    TargetCorrectionDirective update(const VehicleState2D& state,
                                     const Vec2d& original_goal,
                                     const LocalObservation& current_patch,
                                     const LocalObservation& local_history,
                                     uint64_t tick);

    /// Reset per-episode correction state (new task / scene).  The
    /// directive update_event restarts at 0 for the fresh episode.
    void reset();

    /// Clear only goal-dependent correction memory while preserving all
    /// monotonic diagnostic/update counters.  Used for a formally accepted
    /// final-goal revision inside the same simulation episode.
    void resetForNewGoal();

    /// Bump and return the directive update event.  Called by the FSM when
    /// a NEW final navigation goal is formally accepted (a goal change is
    /// a directive change, but mission_revision is the only reset trigger
    /// for the 30 Hz planner).
    uint64_t bumpDirectiveEvent();

    // ── Diagnostics (read by the FSM / GUI / logger) ────────────────
    const AvoidanceObservability& lastObservability() const {
        return last_obs_;
    }
    bool correctionActive() const { return correction_active_; }
    SideSelection lockedSide() const { return locked_side_; }
    uint64_t correctionEnterEvent() const { return correction_enter_event_; }
    uint64_t correctionExitEvent() const { return correction_exit_event_; }
    uint64_t correctionUpdateEvent() const { return correction_update_event_; }
    uint64_t directiveUpdateEvent() const { return update_event_; }
    const TargetCorrectionDirective& lastDirective() const {
        return last_directive_;
    }
    /// Remaining re-entry hysteresis guard in 30 Hz TICKS (0 = free).
    int reentryGuardRemaining(uint64_t tick) const {
        return tick < reentry_guard_until_tick_
                   ? static_cast<int>(reentry_guard_until_tick_ - tick)
                   : 0;
    }

private:
    // ── Local observability judgement (decaying local history) ─────
    AvoidanceObservability assessObservability(
        const VehicleState2D& state, const Vec2d& goal,
        const LocalObservation& patch) const;

    // known-free + clearance grid over the decaying local history map.
    //   free[]      = 1 iff the cell is retained as known-FREE;
    //   blocked[]   = 1 iff the cell is within the shared static handoff
    //                 clearance of an observed OCCUPIED cell (the maximum
    //                 of hard, nominal and configured geometric margins);
    //   reachable[] = corner-safe 8-connected flood fill over known-FREE,
    //                 clearance-valid cells from the vehicle cell (apart
    //                 from the bounded start-recovery prefix; OCCUPIED /
    //                 UNKNOWN are never crossed).
    struct LocalFreeGrid {
        double resolution = 0.1;
        int width = 0, height = 0;
        Vec2d origin{0.0, 0.0};  // world position of cell (0,0)
        std::vector<uint8_t> free;
        std::vector<uint8_t> blocked;
        std::vector<uint8_t> reachable;
        bool valid() const { return width > 0 && height > 0; }
        bool inGrid(int ix, int iy) const {
            return ix >= 0 && iy >= 0 && ix < width && iy < height;
        }
        size_t idx(int ix, int iy) const {
            return static_cast<size_t>(iy) * width + ix;
        }
        bool freeAt(const Vec2d& p) const {
            const GridIndex2D g = worldToGrid(p, origin, resolution);
            if (!inGrid(g.ix, g.iy)) return false;
            return free[idx(g.ix, g.iy)] != 0;
        }
        bool traversableAt(const Vec2d& p) const {
            const GridIndex2D g = worldToGrid(p, origin, resolution);
            if (!inGrid(g.ix, g.iy)) return false;
            return free[idx(g.ix, g.iy)] != 0 &&
                   blocked[idx(g.ix, g.iy)] == 0;
        }
        bool reachableAt(const Vec2d& p) const {
            const GridIndex2D g = worldToGrid(p, origin, resolution);
            if (!inGrid(g.ix, g.iy)) return false;
            return reachable[idx(g.ix, g.iy)] != 0;
        }
    };
    LocalFreeGrid buildLocalFreeGrid(const VehicleState2D& state,
                                     const LocalObservation& patch) const;

    /// One certified local frontier / bypass candidate on a side.
    struct SideCandidate {
        Vec2d endpoint{0.0, 0.0};
        double bearing = 0.0;  // body-frame bearing (rad)
        double dist = 0.0;
        double along_progress = 0.0;  // projection onto goal axis (m)
        bool certified = false;
    };
    /// Deterministically sample and certify side candidates.
    ///   strict=true  → genuine bypass-exit test: the endpoint must lie
    ///                  PAST the nearest blocker surface (positive goal
    ///                  progress beyond the blocker), be in-FOV, connected
    ///                  through known-free space at the shared static
    ///                  handoff clearance, differ from the original-goal
    ///                  direction by no more than FOV/2, and NOT be
    ///                  truncated by UNKNOWN / FOV boundary;
    ///   strict=false → ordinary observation-frontier test (no
    ///                  beyond-blocker requirement), used by
    ///                  NORMAL_CORRECTION.
    std::vector<SideCandidate> sampleSideCandidates(
        const VehicleState2D& state, const Vec2d& goal,
        const LocalObservation& patch, const LocalFreeGrid& grid,
        bool has_blocker, double blocker_min_along, SideSelection side,
        bool strict) const;

    /// Whole chord vehicle→endpoint known-FREE in the current patch, and
    /// grid-traversable (drone clearance) beyond the short recovery prefix.
    bool chordClear(const VehicleState2D& state,
                    const LocalObservation& patch, const LocalFreeGrid& grid,
                    const Vec2d& endpoint) const;

    /// Free range (m) along a WORLD bearing from an arbitrary point until
    /// the first non-FREE cell (current patch; OCCUPIED/UNKNOWN stop it).
    double freeRangeAlongFrom(const LocalObservation& obs, const Vec2d& from,
                              double bearing_world) const;
    /// Free range (m) along a BODY bearing from the vehicle.
    double freeRangeAlong(const VehicleState2D& state,
                          const LocalObservation& obs,
                          double bearing_body) const;

    /// Extract the corridor-blocking OCCUPIED cluster geometry from the
    /// current patch: nearest forward distance and max corridor lateral
    /// extent of the cells inside the vehicle→goal swept corridor.
    bool extractBlocker(const VehicleState2D& state, const Vec2d& goal,
                        const LocalObservation& patch,
                        double& blocker_min_along,
                        double& blocker_max_lateral) const;

    /// Side selection from current-patch visible evidence only
    /// (ambiguous → fixed RIGHT).
    SideSelection selectSide(const VehicleState2D& state,
                             const LocalObservation& patch,
                             const Vec2d& goal) const;

    /// Build the directive for an ACTIVE correction on the locked side:
    /// NORMAL_CORRECTION when a safe ordinary observation frontier exists
    /// (expert bearing quantized through the student adapter, distance
    /// clamped to normal_distance_max, world point rebuilt from the
    /// quantized values), otherwise TURN_LEFT / TURN_RIGHT (fixed
    /// FOV-external ray, normalized_distance = 1.0, no locked world point).
    TargetCorrectionDirective makeCorrectionDirective(
        const VehicleState2D& state, const Vec2d& goal,
        const LocalObservation& patch, const LocalFreeGrid& grid,
        SideSelection side) const;

    /// True when the directive VALUE changed beyond tolerance (type / side
    /// / NORMAL_CORRECTION world point / bounded TURN world anchor).
    bool directiveChanged(const TargetCorrectionDirective& a,
                          const TargetCorrectionDirective& b) const;

    Params2D p_;
    EffectiveTargetAdapter adapter_;
    // ── 5 Hz internal memory (never anything from the 30 Hz outcome) ──
    bool correction_active_ = false;
    SideSelection locked_side_ = SideSelection::NONE;
    int enter_stable_count_ = 0;
    // Re-entry hysteresis, unit = 30 Hz TICKS (after a correction exit the
    // corrector refuses to re-enter until this tick).
    uint64_t reentry_guard_until_tick_ = 0;
    uint64_t update_event_ = 0;
    uint64_t correction_enter_event_ = 0;
    uint64_t correction_exit_event_ = 0;
    uint64_t correction_update_event_ = 0;
    TargetCorrectionDirective last_directive_;
    AvoidanceObservability last_obs_;
};

}  // namespace il_2d_multiscale_debug
