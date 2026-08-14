#include "il_2d_multiscale_debug/left_right_route_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <tuple>

namespace il_2d_multiscale_debug {

// ────────────────────────────────────────────────────────────────────
//  A* on the grid with optional side-bias potential
// ────────────────────────────────────────────────────────────────────
std::vector<Vec2d> LeftRightRoutePlanner::astarPath(
    const TruthEsdf2D& esdf, double clearance, const Vec2d& a, const Vec2d& b,
    const Vec2d& side_axis, const Vec2d& side_center, double side_bias,
    int side_sign) const {
    if (esdf.width() == 0) return {};
    const int sx = esdf.ixOf(a.x()), sy = esdf.iyOf(a.y());
    const int gx = esdf.ixOf(b.x()), gy = esdf.iyOf(b.y());
    if (!esdf.cellFree(sx, sy, clearance) || !esdf.cellFree(gx, gy, clearance))
        return {};

    const int W = esdf.width(), H = esdf.height();
    const double res = esdf.resolution();
    const double axis_len = std::max(1e-6, side_axis.norm());

    const int di[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dj[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    const double w[8] = {1.0, M_SQRT2, 1.0, M_SQRT2, 1.0, M_SQRT2, 1.0, M_SQRT2};

    std::vector<double> gcost(static_cast<size_t>(W) * H,
                              std::numeric_limits<double>::infinity());
    std::vector<int> parent(static_cast<size_t>(W) * H, -1);
    std::vector<bool> closed(static_cast<size_t>(W) * H, false);

    auto h = [&](int ix, int iy) {
        const double dx = (ix - gx) * res, dy = (iy - gy) * res;
        return std::sqrt(dx * dx + dy * dy);
    };
    auto sidePenalty = [&](int ix, int iy) -> double {
        if (side_bias <= 0.0) return 0.0;
        const Vec2d p(esdf.min_bounds().x() + (ix + 0.5) * res,
                      esdf.min_bounds().y() + (iy + 0.5) * res);
        // Unified convention:  cross(axis, p - a) > 0  ⇔  LEFT, where
        // `axis` is the FIXED side axis (never the per-call direction).
        const double lateral =
            cross2(side_axis, p - side_center) / axis_len;
        // LEFT(side_sign=+1): penalize the RIGHT side (lateral < 0).
        // RIGHT(side_sign=-1): penalize the LEFT side (lateral > 0).
        return side_bias * std::max(0.0, -side_sign * lateral);
    };

    using Node = std::tuple<double, double, int, int>;  // f, g, ix, iy
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    const size_t sid = static_cast<size_t>(sy) * W + sx;
    gcost[sid] = 0.0;
    open.emplace(h(sx, sy) + sidePenalty(sx, sy), 0.0, sx, sy);

    bool found = false;
    int final_ix = -1, final_iy = -1;
    while (!open.empty()) {
        const auto [f, g, cx, cy] = open.top();
        open.pop();
        const size_t cid = static_cast<size_t>(cy) * W + cx;
        if (closed[cid]) continue;
        closed[cid] = true;
        if (cx == gx && cy == gy) {
            found = true;
            final_ix = cx;
            final_iy = cy;
            break;
        }
        for (int k = 0; k < 8; ++k) {
            const int nx = cx + dj[k], ny = cy + di[k];
            if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
            if (!esdf.cellFree(nx, ny, clearance)) continue;
            // No diagonal corner cutting (consistent with connectivity).
            if ((k % 2 == 1) &&
                (!esdf.cellFree(cx + dj[k], cy, clearance) ||
                 !esdf.cellFree(cx, cy + di[k], clearance))) {
                continue;
            }
            const size_t nid = static_cast<size_t>(ny) * W + nx;
            if (closed[nid]) continue;
            // The side penalty enters the ACCUMULATED g-cost, not just the
            // priority f value.
            const double ng = g + w[k] * res + sidePenalty(nx, ny) * res;
            if (ng < gcost[nid] - 1e-9) {
                gcost[nid] = ng;
                parent[nid] = static_cast<int>(cid);
                open.emplace(ng + h(nx, ny), ng, nx, ny);
            }
        }
    }
    if (!found) return {};

    std::vector<Vec2d> path;
    int ix = final_ix, iy = final_iy;
    while (ix >= 0 && iy >= 0) {
        path.push_back(Vec2d(esdf.min_bounds().x() + (ix + 0.5) * res,
                             esdf.min_bounds().y() + (iy + 0.5) * res));
        const size_t id = static_cast<size_t>(iy) * W + ix;
        const int p = parent[id];
        if (p < 0) break;
        ix = p % W;
        iy = p / W;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// ────────────────────────────────────────────────────────────────────
//  Tangent gateway on the requested GLOBAL side
// ────────────────────────────────────────────────────────────────────
Vec2d LeftRightRoutePlanner::sideTangent(const Vec2d& p, const Vec2d& center,
                                         double R, const Vec2d& axis,
                                         SideSelection side) const {
    // side normals relative to the start→goal AXIS (not to p)
    const Vec2d n(-axis.y(), axis.x());  // left normal
    const Vec2d u = p - center;
    const double d = u.norm();
    if (d <= R) {
        // point inside/on the inflated circle → radial side-normal point
        const Vec2d nn = (side == SideSelection::LEFT) ? n : -n;
        return center + nn * R;
    }
    const Vec2d dir = u / d;
    const double alpha = std::acos(clamp(R / d, -1.0, 1.0));
    const Vec2d t1 = center + rot2(dir, +alpha) * R;
    const Vec2d t2 = center + rot2(dir, -alpha) * R;
    // Choose the tangent whose sign of cross(axis, t - blocker.center)
    // matches the requested side (>0 LEFT, <0 RIGHT).  The SAME rotation
    // sign is never applied to both start and goal.
    const double l1 = cross2(axis, t1 - center);
    const double l2 = cross2(axis, t2 - center);
    if (side == SideSelection::LEFT) return (l1 > l2) ? t1 : t2;
    return (l1 < l2) ? t1 : t2;
}

Vec2d LeftRightRoutePlanner::projectToLegalCell(const TruthEsdf2D& esdf,
                                                 double clearance,
                                                 const Vec2d& pt) const {
    const double res = esdf.resolution();
    const double eps = 0.5 * res * std::sqrt(2.0) + 1e-3;
    // Margin ≥ half the cell diagonal (+ a small ε) so the projected
    // point is unambiguously interior to a legal cell under the strict
    // `>` comparison.
    const int max_ring = std::max(
        1, static_cast<int>(std::ceil(
               p_.macro_gateway_projection_radius_m / std::max(1e-6, res))));
    for (int ring = 1; ring <= max_ring; ++ring) {
        for (int dy = -ring; dy <= ring; ++dy) {
            for (int dx = -ring; dx <= ring; ++dx) {
                if (std::max(std::abs(dx), std::abs(dy)) != ring) continue;
                const Vec2d c(pt.x() + dx * res, pt.y() + dy * res);
                if (esdf.isFree(c, clearance + eps)) return c;
            }
        }
    }
    // No legal neighbour within the bounded spiral — return the point
    // unchanged; the caller's fallback (plain side-A*) still runs and the
    // homotopy check protects the side rule.
    return pt;
}

// ────────────────────────────────────────────────────────────────────
//  Route construction
// ────────────────────────────────────────────────────────────────────
bool LeftRightRoutePlanner::straightSafe(const TruthEsdf2D& esdf,
                                         double clearance, const Vec2d& a,
                                         const Vec2d& b) const {
    const double dist = (b - a).norm();
    const int steps = std::max(2, static_cast<int>(std::ceil(dist / (0.5 * esdf.resolution()))));
    for (int i = 0; i <= steps; ++i) {
        const Vec2d p = a + (b - a) * (static_cast<double>(i) / steps);
        if (!esdf.isFree(p, clearance)) return false;
    }
    return true;
}

void LeftRightRoutePlanner::losShortcut(const TruthEsdf2D& esdf, double clearance,
                                        std::vector<Vec2d>& path) const {
    bool improved = true;
    while (improved && path.size() > 2) {
        improved = false;
        for (size_t i = 0; i + 2 < path.size(); ++i) {
            for (size_t j = path.size() - 1; j > i + 1; --j) {
                if (straightSafe(esdf, clearance, path[i], path[j])) {
                    path.erase(path.begin() + static_cast<long>(i) + 1,
                               path.begin() + static_cast<long>(j));
                    improved = true;
                    break;
                }
            }
            if (improved) break;
        }
    }
}

void LeftRightRoutePlanner::smooth(const TruthEsdf2D& esdf, double clearance,
                                   std::vector<Vec2d>& path) const {
    if (path.size() < 3) return;
    const int iterations = 80;
    for (int it = 0; it < iterations; ++it) {
        for (size_t i = 1; i + 1 < path.size(); ++i) {
            const Vec2d mid = (path[i - 1] + path[i + 1]) * 0.5;
            const Vec2d cand = path[i] + (mid - path[i]) * 0.5;
            if (esdf.isFree(cand, clearance)) path[i] = cand;
        }
    }
}

bool LeftRightRoutePlanner::routeSafe(const TruthEsdf2D& esdf, double clearance,
                                      const std::vector<Vec2d>& path,
                                      double recovery_prefix_length) const {
    if (path.empty()) return false;
    const double base = p_.scene_safety_clearance;
    const double res = std::max(1e-3, esdf.resolution());
    double acc = 0.0;
    // Every ADJACENT segment is sampled continuously at ≤ res/2.  Each
    // sample uses the BASE clearance when it lies inside the recovery
    // prefix arc length, otherwise the route clearance.
    for (size_t i = 1; i < path.size(); ++i) {
        const Vec2d& a = path[i - 1];
        const Vec2d& b = path[i];
        const double seg = (b - a).norm();
        const int steps = std::max(2, static_cast<int>(std::ceil(seg / (0.5 * res))));
        for (int k = 0; k <= steps; ++k) {
            const Vec2d p = a + (b - a) * (static_cast<double>(k) / steps);
            const double s = acc + seg * (static_cast<double>(k) / steps);
            const double cl =
                (recovery_prefix_length > 0.0 && s <= recovery_prefix_length + 1e-9)
                    ? base
                    : clearance;
            if (!esdf.isFree(p, cl)) return false;
        }
        acc += seg;
    }
    return true;
}

// ────────────────────────────────────────────────────────────────────
//  Start-clearance recovery (§9)
// ────────────────────────────────────────────────────────────────────
bool LeftRightRoutePlanner::findStartRecoveryCell(const TruthEsdf2D& esdf,
                                                  double clearance,
                                                  const Vec2d& start,
                                                  Vec2d& recovery_cell) const {
    const double res = esdf.resolution();
    if (res <= 0.0 || esdf.width() == 0) return false;
    const int max_ring = std::max(
        1, static_cast<int>(std::ceil(
               p_.macro_start_recovery_max_radius_m / std::max(1e-6, res))));
    const int ix0 = esdf.ixOf(start.x());
    const int iy0 = esdf.iyOf(start.y());
    // Search every cell in the finite disk and select the nearest
    // route-legal cell whose connector is continuously safe at the base
    // clearance.  Returning the first ring cell is insufficient because
    // that connector may be blocked while another nearby one is valid.
    const double max_radius = p_.macro_start_recovery_max_radius_m;
    double best_dist = std::numeric_limits<double>::infinity();
    int best_ix = std::numeric_limits<int>::max();
    int best_iy = std::numeric_limits<int>::max();
    for (int dy = -max_ring; dy <= max_ring; ++dy) {
        for (int dx = -max_ring; dx <= max_ring; ++dx) {
            if (dx == 0 && dy == 0) continue;
            const int ix = ix0 + dx;
            const int iy = iy0 + dy;
            if (!esdf.cellFree(ix, iy, clearance)) continue;
            const Vec2d c(esdf.min_bounds().x() + (ix + 0.5) * res,
                          esdf.min_bounds().y() + (iy + 0.5) * res);
            const double d = (c - start).norm();
            if (d > max_radius + 1e-9) continue;
            if (!straightSafe(esdf, p_.scene_safety_clearance, start, c))
                continue;
            if (d < best_dist - 1e-9 ||
                (std::fabs(d - best_dist) <= 1e-9 &&
                 std::tie(iy, ix) < std::tie(best_iy, best_ix))) {
                best_dist = d;
                best_ix = ix;
                best_iy = iy;
                recovery_cell = c;
            }
        }
    }
    return std::isfinite(best_dist);
}

void LeftRightRoutePlanner::prependRecovery(
    const Vec2d& start, const Vec2d& route_start, const TruthEsdf2D& esdf,
    std::vector<Vec2d>& path) const {
    if (path.empty()) return;
    // The recovery connection is a straight segment start → route_start.
    // `path` already begins at route_start; drop that duplicate point so it
    // is not emitted twice.
    if (!path.empty() && (path.front() - route_start).norm() < 1e-9) {
        path.erase(path.begin());
    }
    const double seg = (route_start - start).norm();
    const int steps =
        std::max(1, static_cast<int>(std::ceil(seg / esdf.resolution())));
    std::vector<Vec2d> full;
    full.reserve(path.size() + steps + 1);
    full.push_back(start);
    for (int k = 1; k <= steps; ++k) {
        full.push_back(start + (route_start - start) *
                                   (static_cast<double>(k) / steps));
    }
    full.insert(full.end(), path.begin(), path.end());
    path.swap(full);
}

// ────────────────────────────────────────────────────────────────────
//  Homotopy verification (FIXED side reference, continuous)
// ────────────────────────────────────────────────────────────────────
bool LeftRightRoutePlanner::passesBlockerOnSide(
    const Vec2d& side_axis, const Vec2d& side_center,
    const BlockerInfo& blocker, SideSelection side,
    const std::vector<Vec2d>& path) const {
    if (!blocker.found || path.empty()) return true;
    const double axis_len = std::max(1e-6, side_axis.norm());
    const double tol = p_.macro_homotopy_side_tolerance_m;
    const double infl = blocker.radius + p_.scene_safety_clearance +
                        p_.macro_route_clearance_margin +
                        p_.lp_clearance_discretization_margin_m;
    // Continuous segment sampling at ≤ ~esdf_resolution/2 (0.05 m).
    const double step = std::max(1e-3, 0.5 * p_.esdf_resolution);

    double nearest_lat = 0.0;
    double nearest_d = std::numeric_limits<double>::infinity();
    for (size_t i = 1; i < path.size(); ++i) {
        const Vec2d& a = path[i - 1];
        const Vec2d& b = path[i];
        const double seg = (b - a).norm();
        const int steps = std::max(1, static_cast<int>(std::ceil(seg / step)));
        for (int k = 0; k <= steps; ++k) {
            const Vec2d p = a + (b - a) * (static_cast<double>(k) / steps);
            const double d = (p - side_center).norm();
            // cross(side_axis, p - side_center) > 0  ⇔  LEFT (FIXED ref)
            const double lat = cross2(side_axis, p - side_center) / axis_len;
            if (d < nearest_d) {
                nearest_d = d;
                nearest_lat = lat;
            }
            if (d < infl) {
                if (side == SideSelection::LEFT && lat < -tol) return false;
                if (side == SideSelection::RIGHT && lat > tol) return false;
            }
        }
    }
    const bool passes = (side == SideSelection::LEFT) ? nearest_lat > tol
                                                      : nearest_lat < -tol;
    return passes;
}

Route2D LeftRightRoutePlanner::densify(const std::vector<Vec2d>& path,
                                       double spacing) const {
    Route2D out;
    if (path.empty()) return out;
    out.waypoints.push_back(path.front());
    for (size_t i = 1; i < path.size(); ++i) {
        const Vec2d a = path[i - 1], b = path[i];
        const double seg = (b - a).norm();
        // Walk the segment at `spacing` steps (do not re-emit the first
        // point, it is already the previous segment's end).
        for (double d = spacing; d < seg - 1e-9; d += spacing) {
            out.waypoints.push_back(a + (b - a) * (d / seg));
        }
        out.waypoints.push_back(b);
        out.length += seg;
    }
    return out;
}

bool LeftRightRoutePlanner::buildSideRoute(
    const Scene2D& scene, const TruthEsdf2D& esdf, const Vec2d& start,
    const Vec2d& goal, const BlockerInfo& blocker, SideSelection side,
    const HomotopyReference* ref, Route2D& out) const {
    const double clearance =
        p_.scene_safety_clearance + p_.macro_route_clearance_margin +
        p_.lp_clearance_discretization_margin_m;
    const double base = p_.scene_safety_clearance;
    (void)scene;
    const int side_sign = (side == SideSelection::LEFT) ? 1 : -1;
    const double bias = p_.macro_route_side_bias;

    // FIXED physical homotopy reference: at macro runtime the axis and the
    // blocker centre are the ones captured at macro entry, so vehicle
    // movement / goal changes never redefine LEFT/RIGHT.  Task
    // qualification (ref == nullptr) uses the current start→goal axis.
    Vec2d axis = goal - start;
    Vec2d center = blocker.center;
    if (ref && ref->valid) {
        axis = ref->entry_axis;
        center = ref->entry_blocker_center;
    }
    const double axis_len = std::max(1e-6, axis.norm());
    const Vec2d axis_u = axis / axis_len;

    // ── Start-clearance recovery (§9) ───────────────────────────────
    // If the macro A* start cell falls short of the ROUTE clearance
    // (0.1 m discretisation error) while the CONTINUOUS start is still
    // above scene_safety_clearance, find the nearest route-clear legal
    // cell inside a bounded radius and prepend a short recovery connection
    // (start → recovery cell) that satisfies ONLY the base clearance.  The
    // rest of the route still satisfies the same macro/local handoff
    // clearance, including the discretisation margin.  This never silently
    // changes the LEFT/RIGHT
    // side (the search is side-neutral) and never skips the start
    // clearance check entirely.
    Vec2d route_start = start;
    Vec2d recovery_cell(0.0, 0.0);
    double recovery_len = 0.0;
    bool recovery_used = false;
    const bool start_cell_route_free = esdf.cellFree(
        esdf.ixOf(start.x()), esdf.iyOf(start.y()), clearance);
    if (!start_cell_route_free && esdf.isFree(start, base)) {
        if (findStartRecoveryCell(esdf, clearance, start, recovery_cell)) {
            // The recovery connection must itself be base-safe (never cut
            // through an obstacle / a below-base-clearance cell).
            if (straightSafe(esdf, base, start, recovery_cell)) {
                route_start = recovery_cell;
                recovery_len = (recovery_cell - start).norm();
                recovery_used = true;
            }
        }
    }

    std::vector<Vec2d> path;
    if (blocker.found) {
        // MUST-pass side gateways relative to the FIXED reference
        // (side_axis, side_center): both tangent points are computed and
        // the one on the requested global side is chosen; the rotation
        // sign is NEVER shared between start and goal.  The A* start is
        // `route_start` (the recovery cell when a recovery is used).
        const double R = blocker.radius + p_.scene_safety_clearance +
                         p_.macro_route_clearance_margin +
                         p_.lp_clearance_discretization_margin_m;
        const Vec2d gP = sideTangent(start, center, R, axis_u, side);
        const Vec2d gG = sideTangent(goal, center, R, axis_u, side);
        // Tangent points sit exactly at esdf == clearance → project them
        // into STRICTLY legal cells so the A* endpoints pass isFree().
        const Vec2d gP_legal = projectToLegalCell(esdf, clearance, gP);
        const Vec2d gG_legal = projectToLegalCell(esdf, clearance, gG);

        auto s1 = astarPath(esdf, clearance, route_start, gP_legal, axis,
                            center, bias, side_sign);
        auto s2 = astarPath(esdf, clearance, gP_legal, gG_legal, axis, center,
                            bias, side_sign);
        auto s3 = astarPath(esdf, clearance, gG_legal, goal, axis, center,
                            bias, side_sign);
        if (!s1.empty() && !s2.empty() && !s3.empty()) {
            // Per-segment shortcut + smooth so the forced gateways are
            // NEVER shortcut away and the homotopy side cannot flip.
            losShortcut(esdf, clearance, s1);
            losShortcut(esdf, clearance, s2);
            losShortcut(esdf, clearance, s3);
            if (routeSafe(esdf, clearance, s1) &&
                routeSafe(esdf, clearance, s2) &&
                routeSafe(esdf, clearance, s3)) {
                const std::vector<Vec2d> p1 = s1, p2 = s2, p3 = s3;
                smooth(esdf, clearance, s1);
                smooth(esdf, clearance, s2);
                smooth(esdf, clearance, s3);
                if (!routeSafe(esdf, clearance, s1)) s1 = p1;  // rollback
                if (!routeSafe(esdf, clearance, s2)) s2 = p2;
                if (!routeSafe(esdf, clearance, s3)) s3 = p3;
                path = s1;
                path.insert(path.end(), s2.begin() + 1, s2.end());
                path.insert(path.end(), s3.begin() + 1, s3.end());
            }
        }
    }
    if (path.empty()) {
        // Fallback: single side-constrained A* (no forced gateways).
        path = astarPath(esdf, clearance, route_start, goal, axis, center,
                         bias, side_sign);
        if (!path.empty()) {
            losShortcut(esdf, clearance, path);
            if (routeSafe(esdf, clearance, path)) {
                const std::vector<Vec2d> pre = path;
                smooth(esdf, clearance, path);
                if (!routeSafe(esdf, clearance, path)) path = pre;
            } else {
                path.clear();
            }
        }
    }
    if (path.empty()) return false;

    // Prepend the recovery connection (verified at BASE clearance) and
    // re-verify the FULL path with the explicit recovery-prefix split.
    if (recovery_used) {
        prependRecovery(start, route_start, esdf, path);
        if (!routeSafe(esdf, clearance, path, recovery_len)) return false;
    }

    // Homotopy: the path must actually pass the blocker on the requested
    // side (continuous check against the FIXED reference).  If LEFT/RIGHT
    // collapse onto the same homotopy, the side is reported invalid — the
    // two identical routes are never shown as two.
    if (blocker.found &&
        !passesBlockerOnSide(axis, center, blocker, side, path)) {
        return false;
    }

    out = densify(path, esdf.resolution());
    out.recovery_prefix_length = recovery_used ? recovery_len : 0.0;
    out.start_clearance_recovery_used = recovery_used;
    out.valid = out.waypoints.size() >= 2;
    return out.valid;
}

RoutePlanResult LeftRightRoutePlanner::planRoutes(
    const Scene2D& scene, const TruthEsdf2D& esdf, const Vec2d& start,
    const Vec2d& goal, const BlockerInfo& blocker,
    const HomotopyReference* ref) {
    RoutePlanResult res;
    res.left_valid = buildSideRoute(scene, esdf, start, goal, blocker,
                                    SideSelection::LEFT, ref, res.left);
    res.right_valid = buildSideRoute(scene, esdf, start, goal, blocker,
                                     SideSelection::RIGHT, ref, res.right);
    return res;
}

Route2D LeftRightRoutePlanner::planPlainRoute(const TruthEsdf2D& esdf,
                                              const Vec2d& start,
                                              const Vec2d& goal) const {
    const double clearance =
        p_.scene_safety_clearance + p_.macro_route_clearance_margin +
        p_.lp_clearance_discretization_margin_m;
    const double base = p_.scene_safety_clearance;
    // No side bias, no gateways, no homotopy constraint — a plain global
    // ESDF route used to keep generating guidance once the blocker is
    // passed.  Never fails the task on infeasibility.
    // Same defensive start-clearance recovery as buildSideRoute.
    Vec2d route_start = start;
    Vec2d recovery_cell(0.0, 0.0);
    double recovery_len = 0.0;
    bool recovery_used = false;
    const bool start_cell_route_free = esdf.cellFree(
        esdf.ixOf(start.x()), esdf.iyOf(start.y()), clearance);
    if (!start_cell_route_free && esdf.isFree(start, base)) {
        if (findStartRecoveryCell(esdf, clearance, start, recovery_cell) &&
            straightSafe(esdf, base, start, recovery_cell)) {
            route_start = recovery_cell;
            recovery_len = (recovery_cell - start).norm();
            recovery_used = true;
        }
    }
    std::vector<Vec2d> path =
        astarPath(esdf, clearance, route_start, goal, Vec2d::UnitX(), start,
                  0.0, 1);
    Route2D out;
    if (path.empty()) return out;
    losShortcut(esdf, clearance, path);
    if (!routeSafe(esdf, clearance, path)) {
        // Plain route unsafe — do not force it; report invalid (caller
        // falls back to the goal as the local target).
        return out;
    }
    const std::vector<Vec2d> pre = path;
    smooth(esdf, clearance, path);
    if (!routeSafe(esdf, clearance, path)) path = pre;
    if (recovery_used) {
        prependRecovery(start, route_start, esdf, path);
        if (!routeSafe(esdf, clearance, path, recovery_len)) return out;
    }
    out = densify(path, esdf.resolution());
    out.recovery_prefix_length = recovery_used ? recovery_len : 0.0;
    out.start_clearance_recovery_used = recovery_used;
    out.valid = out.waypoints.size() >= 2;
    return out;
}

}  // namespace il_2d_multiscale_debug
