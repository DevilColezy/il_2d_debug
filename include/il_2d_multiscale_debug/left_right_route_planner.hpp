#pragma once
/// @file   left_right_route_planner.hpp
/// @brief  Privileged LEFT / RIGHT detour route generation around the
///         blocker (tangent gateways + A* + LOS shortcut + smoothing).
///
/// PRIVILEGED: reads the global truth ESDF and the true obstacle list.
/// It is used ONLY by the 5 Hz macro expert to build/refresh the route
/// for the side that was ALREADY chosen from visible evidence, and to
/// display both routes to the human.  Its outputs are never delivered to
/// the 30 Hz planner.
///
/// Both homotopy branches are computed so the GUI can show LEFT and
/// RIGHT; the side actually chosen is decided elsewhere (MacroExpert5Hz)
/// strictly from visible evidence, never from route length/cost.

#include "il_2d_multiscale_debug/truth_esdf_2d.hpp"
#include "il_2d_multiscale_debug/types.hpp"

namespace il_2d_multiscale_debug {

struct RoutePlanResult {
    Route2D left;
    Route2D right;
    bool left_valid = false;
    bool right_valid = false;
};

class LeftRightRoutePlanner {
public:
    explicit LeftRightRoutePlanner(const Params2D& p) : p_(p) {}

    /// Build both routes around `blocker` (center + inflated radius).
    /// The gateway / side-bias / homotopy geometry uses the FIXED
    /// homotopy reference `ref` when it is valid (macro runtime: the
    /// vehicle can move and the goal can change without redefining
    /// LEFT/RIGHT).  When `ref` is null or invalid (task qualification)
    /// the current start→goal axis and the blocker centre are used.
    RoutePlanResult planRoutes(const Scene2D& scene, const TruthEsdf2D& esdf,
                               const Vec2d& start, const Vec2d& goal,
                               const BlockerInfo& blocker,
                               const HomotopyReference* ref = nullptr);

    /// Plain global-ESDF route from `start` to `goal` — NO forced
    /// gateways, NO homotopy constraint.  Used for guidance once the
    /// blocker is passed (blocker_passed_latched=true).  An infeasible
    /// plain route never fails the task (caller falls back to the goal).
    Route2D planPlainRoute(const TruthEsdf2D& esdf, const Vec2d& start,
                           const Vec2d& goal) const;

private:
    /// A* on grid cells with esdf > clearance.  side_bias>0 pushes the
    /// path to the requested side.  `axis` is the FIXED side axis used
    /// for the side-penalty (never the per-call a→b direction when a
    /// fixed reference exists). `side_center` is the same fixed blocker
    /// centre, so sub-route endpoints cannot translate the side split.
    /// side_sign: +1 LEFT, -1 RIGHT.
    std::vector<Vec2d> astarPath(const TruthEsdf2D& esdf, double clearance,
                                 const Vec2d& a, const Vec2d& b,
                                 const Vec2d& side_axis,
                                 const Vec2d& side_center,
                                 double side_bias, int side_sign) const;

    /// Tangent point from P to the inflated circle (center, R) that lies
    /// on the requested GLOBAL side.  Both tangents are computed and the
    /// one with the matching sign of cross(axis, p - blocker.center) is
    /// chosen — the rotation sign is NEVER shared between start and goal.
    Vec2d sideTangent(const Vec2d& p, const Vec2d& center, double R,
                      const Vec2d& axis, SideSelection side) const;

    /// The tangent gateway sits exactly at esdf == clearance, where the
    /// STRICT `>` test of isFree() fails for the A* endpoints.  This
    /// pushes the point to the nearest cell that is strictly legal with a
    /// margin ≥ half the cell diagonal (+ε).  Bounded spiral search (8
    /// rings); returns `pt` unchanged if none is found within the limit.
    Vec2d projectToLegalCell(const TruthEsdf2D& esdf, double clearance,
                             const Vec2d& pt) const;

    /// Bounded start-clearance recovery (§9): search the finite-radius disk
    /// around `start` for the nearest route-clear cell whose straight
    /// connector is continuously safe at the BASE clearance.  Used when
    /// the A* start cell is not route-clear while the continuous start is
    /// still legal at scene_safety_clearance.  Returns the cell centre.
    bool findStartRecoveryCell(const TruthEsdf2D& esdf, double clearance,
                               const Vec2d& start, Vec2d& recovery_cell) const;

    /// Prepend the verified recovery connection (start → route_start) to a
    /// route whose A* already starts at `route_start`.  The recovery
    /// segment satisfies ONLY the base safety clearance; everything after
    /// route_start satisfies the route clearance.
    void prependRecovery(const Vec2d& start, const Vec2d& route_start,
                         const TruthEsdf2D& esdf,
                         std::vector<Vec2d>& path) const;

    bool buildSideRoute(const Scene2D& scene, const TruthEsdf2D& esdf,
                        const Vec2d& start, const Vec2d& goal,
                        const BlockerInfo& blocker, SideSelection side,
                        const HomotopyReference* ref, Route2D& out) const;

    /// True iff the path actually passes the blocker on the requested side
    /// (homotopy check), using the FIXED reference:
    ///   cross(side_axis, p - side_center) > 0  ⇔  LEFT.
    /// Samples the CONTINUOUS path (segments at ≤ ~res/2), not just the
    /// discrete waypoints.
    bool passesBlockerOnSide(const Vec2d& side_axis, const Vec2d& side_center,
                             const BlockerInfo& blocker, SideSelection side,
                             const std::vector<Vec2d>& path) const;

    void losShortcut(const TruthEsdf2D& esdf, double clearance,
                     std::vector<Vec2d>& path) const;
    void smooth(const TruthEsdf2D& esdf, double clearance,
                std::vector<Vec2d>& path) const;
    bool straightSafe(const TruthEsdf2D& esdf, double clearance,
                      const Vec2d& a, const Vec2d& b) const;
    /// Verify the whole polyline continuously (≤ res/2 sampling).  When
    /// `recovery_prefix_length` > 0, the first that many metres are checked
    /// at the BASE safety clearance (recovery prefix) and the remainder at
    /// `clearance` (route clearance) — an explicit C++ distinction between
    /// the recovery prefix and the normal route.
    bool routeSafe(const TruthEsdf2D& esdf, double clearance,
                   const std::vector<Vec2d>& path,
                   double recovery_prefix_length = 0.0) const;
    Route2D densify(const std::vector<Vec2d>& path, double spacing) const;

    Params2D p_;
};

}  // namespace il_2d_multiscale_debug
