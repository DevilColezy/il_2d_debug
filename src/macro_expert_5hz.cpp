#include "il_2d_multiscale_debug/macro_expert_5hz.hpp"
#include "il_2d_multiscale_debug/local_planner_30hz.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <unordered_map>
#include <utility>

namespace il_2d_multiscale_debug {

// ────────────────────────────────────────────────────────────────────
//  Visible-prefix side selection (LOCAL OBSERVATION ONLY)
// ────────────────────────────────────────────────────────────────────
double MacroExpert5Hz::freeRangeAlong(const VehicleState2D& state,
                                      const LocalObservation& obs,
                                      double bearing) const {
    const Vec2d dir(std::cos(state.yaw + bearing),
                    std::sin(state.yaw + bearing));
    const double step = obs.resolution * 0.5;
    double range = 0.0;
    for (double d = step; d <= p_.obs_range_m + 1e-9; d += step) {
        const Vec2d p = state.position + dir * d;
        if (!obs.isKnownFree(p.x(), p.y())) break;  // OCCUPIED or UNKNOWN
        range = d;
    }
    return range;
}

SideEvidence MacroExpert5Hz::selectSideFromVisibleEvidence(
    const VehicleState2D& state, const LocalObservation& obs,
    const Vec2d& goal) const {
    const double fov = deg2rad(p_.obs_fov_deg);
    const double beta_goal =
        wrapAngle(std::atan2(goal.y() - state.position.y(),
                             goal.x() - state.position.x()) -
                  state.yaw);
    const double d_beta = deg2rad(p_.macro_evidence_ray_step_deg);

    // PAIRED symmetric rays: for every offset db we evaluate the ray at
    // (beta_goal + db) [LEFT] and (beta_goal - db) [RIGHT], and only count
    // the pair when BOTH rays lie inside the current FOV.  Scores are the
    // AVERAGE visible free range, never an unnormalised sum.
    double left_total = 0.0, right_total = 0.0;
    int pairs = 0;
    for (double db = d_beta; db <= fov / 2.0 - 1e-6; db += d_beta) {
        const double bl = beta_goal + db;  // LEFT (positive bearing)
        const double br = beta_goal - db;  // RIGHT
        if (std::fabs(bl) > fov / 2.0 || std::fabs(br) > fov / 2.0) continue;
        left_total += freeRangeAlong(state, obs, bl);
        right_total += freeRangeAlong(state, obs, br);
        ++pairs;
    }

    SideEvidence ev;
    const double left_avg =
        pairs > 0 ? left_total / static_cast<double>(pairs) : 0.0;
    const double right_avg =
        pairs > 0 ? right_total / static_cast<double>(pairs) : 0.0;
    // Scores are the AVERAGE visible free range actually used for the
    // comparison (never the unnormalised totals).
    ev.left_score = left_avg;
    ev.right_score = right_avg;
    if (pairs < std::max(1, p_.macro_min_evidence_ray_pairs)) {
        // Not enough symmetric ray pairs inside the FOV → ambiguous.
        ev.selection = SideSelection::RIGHT;
        ev.from_visible_evidence = false;
        ev.ambiguous_defaulted_right = true;
        ev.reason = "AMBIGUOUS_DEFAULT_RIGHT";
        return ev;
    }

    const double margin = p_.macro_side_evidence_margin;
    if (left_avg > right_avg + margin) {
        ev.selection = SideSelection::LEFT;
        ev.from_visible_evidence = true;
        ev.reason = "VISIBLE_LEFT_BETTER";
    } else if (right_avg > left_avg + margin) {
        ev.selection = SideSelection::RIGHT;
        ev.from_visible_evidence = true;
        ev.reason = "VISIBLE_RIGHT_BETTER";
    } else {
        // Indistinguishable → fixed default RIGHT.
        ev.selection = SideSelection::RIGHT;
        ev.from_visible_evidence = false;
        ev.ambiguous_defaulted_right = true;
        ev.reason = "AMBIGUOUS_DEFAULT_RIGHT";
    }
    return ev;
}

// ────────────────────────────────────────────────────────────────────
//  LOCAL blocker EVIDENCE (OCCUPIED cluster inside the swept corridor)
//  — observation-only; carries NO truth geometry.
// ────────────────────────────────────────────────────────────────────
LocalBlockerEvidence MacroExpert5Hz::identifyBlocker(
    const VehicleState2D& state, const Vec2d& goal,
    const LocalObservation& obs) const {
    LocalBlockerEvidence ev;
    const Vec2d axis = goal - state.position;
    const double axis_len = std::max(1e-6, axis.norm());

    const double res = obs.resolution;
    const double hw = p_.macro_corridor_half_width;

    // Gather candidate OCCUPIED cells inside the corridor ahead.
    std::vector<std::pair<int, int>> cells;
    for (int iy = 0; iy < obs.height; ++iy) {
        for (int ix = 0; ix < obs.width; ++ix) {
            if (obs.cells[obs.idx(ix, iy)] != CellState::OCCUPIED) continue;
            const Vec2d p(obs.origin.x() + (ix + 0.5) * res,
                          obs.origin.y() + (iy + 0.5) * res);
            const Vec2d rel = p - state.position;
            const double lateral = std::fabs(cross2(rel, axis) / axis_len);
            const double along = rel.dot(axis) / axis_len;
            if (lateral <= hw &&
                along >= -p_.macro_corridor_rear_tolerance_m &&
                along <= p_.obs_range_m) {
                cells.emplace_back(ix, iy);
            }
        }
    }
    if (cells.empty()) return ev;

    // Cluster with EXACT 8-connectivity via a grid-index table — never
    // link cells that are two apart.
    auto keyHash = [](const std::pair<int, int>& k) {
        return static_cast<size_t>(k.first) * 73856093u ^
               static_cast<size_t>(k.second) * 19349663u;
    };
    std::unordered_map<std::pair<int, int>, size_t, decltype(keyHash)> cell_index(
        8, keyHash);
    for (size_t i = 0; i < cells.size(); ++i) cell_index[cells[i]] = i;

    std::vector<int> cell_label(cells.size(), -1);
    int nclusters = 0;
    const int di[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dj[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    for (size_t k = 0; k < cells.size(); ++k) {
        if (cell_label[k] >= 0) continue;
        std::deque<size_t> stack{k};
        cell_label[k] = nclusters;
        while (!stack.empty()) {
            const size_t cur = stack.front();
            stack.pop_front();
            for (int d = 0; d < 8; ++d) {
                const auto it = cell_index.find(
                    {cells[cur].first + dj[d], cells[cur].second + di[d]});
                if (it == cell_index.end()) continue;
                const size_t m = it->second;
                if (cell_label[m] >= 0) continue;
                cell_label[m] = nclusters;
                stack.push_back(m);
            }
        }
        ++nclusters;
    }

    // Pick the cluster that GENUINELY intersects the blocked planning
    // corridor and is nearest in FORWARD distance (not nearest Euclidean):
    //   1) a cluster "blocks the corridor" when its cells span at least
    //      half the corridor half-width laterally (it really crosses the
    //      swept corridor, not just grazes one edge);
    //   2) among blocking clusters, the one with the smallest FORWARD
    //      distance along the goal axis wins (deterministic tie-break by
    //      cluster id).
    int best_cluster = -1;
    double best_forward = std::numeric_limits<double>::infinity();
    for (int cl = 0; cl < nclusters; ++cl) {
        double min_lat = std::numeric_limits<double>::infinity();
        double max_lat = -std::numeric_limits<double>::infinity();
        double min_along = std::numeric_limits<double>::infinity();
        int cnt = 0;
        for (size_t k = 0; k < cells.size(); ++k) {
            if (cell_label[k] != cl) continue;
            const Vec2d p(obs.origin.x() + (cells[k].first + 0.5) * res,
                          obs.origin.y() + (cells[k].second + 0.5) * res);
            const Vec2d rel = p - state.position;
            const double lat = cross2(rel, axis) / axis_len;
            const double along = rel.dot(axis) / axis_len;
            min_lat = std::min(min_lat, lat);
            max_lat = std::max(max_lat, lat);
            min_along = std::min(min_along, along);
            ++cnt;
        }
        if (cnt == 0) continue;
        // Spans at least half the corridor width → truly crosses it.
        const bool blocks =
            (max_lat - min_lat) >=
            hw * p_.macro_blocking_lateral_span_ratio;
        if (blocks && min_along < best_forward) {
            best_forward = min_along;
            best_cluster = cl;
        }
    }
    if (best_cluster < 0) {
        // No cluster crosses the corridor centre region; fall back to the
        // nearest-FORWARD cluster (still never nearest-Euclidean-only).
        double best_along = std::numeric_limits<double>::infinity();
        for (int cl = 0; cl < nclusters; ++cl) {
            double min_along = std::numeric_limits<double>::infinity();
            int cnt = 0;
            for (size_t k = 0; k < cells.size(); ++k) {
                if (cell_label[k] != cl) continue;
                const Vec2d p(obs.origin.x() + (cells[k].first + 0.5) * res,
                              obs.origin.y() + (cells[k].second + 0.5) * res);
                const Vec2d rel = p - state.position;
                min_along = std::min(min_along, rel.dot(axis) / axis_len);
                ++cnt;
            }
            if (cnt > 0 && min_along < best_along) {
                best_along = min_along;
                best_cluster = cl;
            }
        }
    }
    if (best_cluster < 0) return ev;

    // Fill the EVIDENCE only: centroid / bounding radius / cell count and
    // the OCCUPIED cell WORLD coordinates (never truth ids).
    Vec2d csum(0.0, 0.0);
    int ccnt = 0;
    double max_r = 0.0;
    std::vector<Vec2d> cell_pts;
    for (size_t k = 0; k < cells.size(); ++k) {
        if (cell_label[k] != best_cluster) continue;
        const Vec2d p(obs.origin.x() + (cells[k].first + 0.5) * res,
                      obs.origin.y() + (cells[k].second + 0.5) * res);
        csum += p;
        cell_pts.push_back(p);
        ++ccnt;
    }
    const Vec2d centroid = csum / static_cast<double>(std::max(1, ccnt));
    for (const Vec2d& p : cell_pts) {
        max_r = std::max(max_r, (p - centroid).norm() + 0.5 * res);
    }

    ev.found = true;
    ev.cluster_id = best_cluster;
    ev.visible_centroid = centroid;
    ev.visible_radius = max_r;
    ev.visible_cell_count = ccnt;
    ev.visible_cells = std::move(cell_pts);
    return ev;
}

// ────────────────────────────────────────────────────────────────────
//  PRIVILEGED blocker: deterministic cell-support association of the
//  local OCCUPIED cluster to truth cylinders.  One dominant cylinder is
//  matched (never a merged "big circle" that would invent an obstacle
//  closing a passable gap between passable-neighbour cylinders).
// ────────────────────────────────────────────────────────────────────
BlockerInfo MacroExpert5Hz::resolvePrivilegedBlocker(
    const LocalBlockerEvidence& ev, const Scene2D& scene) const {
    BlockerInfo info;
    if (!ev.found) {
        // Macro was triggered (observed blockage) but no local cluster is
        // present in the corridor → explicit NO_MATCH (never a silent
        // plain-A* fallback).  The FSM marks the task invalid.
        info.cluster_id = ev.cluster_id;
        info.found = false;
        info.association = BlockerAssociation::NO_MATCH;
        return info;
    }
    info.cluster_id = ev.cluster_id;

    const double surf_tol = p_.macro_blocker_match_surface_tol_m;

    // Per-cylinder support: number of visible cells whose geometric
    // residual to the cylinder surface/inside is <= surf_tol, plus the
    // mean residual (tie-break) and the distance from the evidence
    // centroid to the cylinder centre (secondary tie-break).
    struct CylMatch {
        const Obstacle2D* o = nullptr;
        int support = 0;
        double residual_sum = 0.0;
        double centroid_dist = std::numeric_limits<double>::infinity();
    };
    std::vector<CylMatch> matches;
    for (const auto& o : scene.obstacles) {
        CylMatch m;
        m.o = &o;
        m.centroid_dist = (o.center - ev.visible_centroid).norm();
        for (const Vec2d& c : ev.visible_cells) {
            const double d = (c - o.center).norm();
            // residual: 0 inside, (surface distance) outside
            const double residual = std::max(0.0, d - o.radius);
            if (residual <= surf_tol) {
                ++m.support;
                m.residual_sum += residual;
            }
        }
        if (m.support > 0) matches.push_back(std::move(m));
    }

    // Deterministic ranking: support desc, mean residual asc, centroid
    // distance asc, obstacle id asc.
    std::sort(matches.begin(), matches.end(),
              [](const CylMatch& a, const CylMatch& b) {
                  if (a.support != b.support) return a.support > b.support;
                  const double ra =
                      a.support > 0 ? a.residual_sum / a.support
                                    : std::numeric_limits<double>::infinity();
                  const double rb =
                      b.support > 0 ? b.residual_sum / b.support
                                    : std::numeric_limits<double>::infinity();
                  if (ra != rb) return ra < rb;
                  if (a.centroid_dist != b.centroid_dist)
                      return a.centroid_dist < b.centroid_dist;
                  return a.o->id < b.o->id;
              });

    if (matches.empty()) {
        // No truth cylinder supports the visible cells.
        info.found = false;
        info.association = BlockerAssociation::NO_MATCH;
        return info;
    }
    const CylMatch& best = matches.front();
    if (best.support < p_.macro_blocker_match_min_cells) {
        // Too few supporting cells for a confident MATCHED.
        info.found = false;
        info.association = BlockerAssociation::NO_MATCH;
        return info;
    }
    // AMBIGUOUS when the second-best is within `ambiguity_ratio` of the
    // best support (two plausible cylinders → cannot attribute uniquely).
    if (matches.size() >= 2) {
        const int second = matches[1].support;
        if (static_cast<double>(second) >=
            p_.macro_blocker_match_ambiguity_ratio * static_cast<double>(best.support)) {
            info.found = false;
            info.association = BlockerAssociation::AMBIGUOUS_MATCH;
            return info;
        }
    }

    // MATCHED: geometry = the single dominant cylinder.
    info.found = true;
    info.association = BlockerAssociation::MATCHED;
    info.center = best.o->center;
    info.radius = best.o->radius;
    info.obstacle_ids.push_back(best.o->id);
    return info;
}

// ────────────────────────────────────────────────────────────────────
//  Route building (privileged) + guidance target
// ────────────────────────────────────────────────────────────────────
void MacroExpert5Hz::buildRoutes(const Scene2D& scene, const TruthEsdf2D& esdf,
                                 const VehicleState2D& state, const Vec2d& goal,
                                 const BlockerInfo& blocker, SideSelection side,
                                 const HomotopyReference* ref, Route2D& left,
                                 Route2D& right, Route2D& locked) {
    LeftRightRoutePlanner rp(p_);
    const auto res = rp.planRoutes(scene, esdf, state.position, goal, blocker,
                                   ref);
    left = res.left;
    right = res.right;
    locked = (side == SideSelection::LEFT) ? res.left : res.right;
}

Route2D MacroExpert5Hz::buildPlainRoute(const TruthEsdf2D& esdf,
                                        const VehicleState2D& state,
                                        const Vec2d& goal) const {
    LeftRightRoutePlanner rp(p_);
    return rp.planPlainRoute(esdf, state.position, goal);
}

LocalTarget MacroExpert5Hz::guidanceTarget(const VehicleState2D& state,
                                           const Route2D& locked,
                                           const Vec2d& goal,
                                           const TruthEsdf2D& esdf,
                                           uint64_t tick,
                                           bool force_advance,
                                           double& lookahead_used,
                                           double& route_progress,
                                           std::string& update_reason) {
    LocalTarget t;
    (void)tick;
    lookahead_used = 0.0;
    route_progress = 0.0;
    update_reason = "fallback_goal";
    t.position = goal;
    t.valid = true;
    t.is_macro_guide = true;
    t.update_event = 0;  // caller sets the event counter
    if (!locked.valid || locked.waypoints.size() < 2) return t;

    // ── Per-segment arc lengths + start arcs ───────────────────────
    const auto& w = locked.waypoints;
    const size_t nseg = w.size() - 1;
    std::vector<double> seg_len(nseg, 0.0);
    std::vector<double> seg_start(nseg + 1, 0.0);
    for (size_t i = 0; i < nseg; ++i) {
        seg_len[i] = (w[i + 1] - w[i]).norm();
        seg_start[i + 1] = seg_start[i] + seg_len[i];
    }
    const double total = seg_start[nseg];
    if (total <= 1e-9) return t;  // degenerate route → goal fallback

    // ── 1) Vehicle projection (continuous arc length) ──────────────
    double progress = 0.0, lateral = 0.0;
    if (!projectOnRoute(w, seg_start, seg_len, state.position, progress,
                        lateral)) {
        return t;  // defensive; the route was validated above
    }
    (void)lateral;
    route_progress = progress;

    // ── 2) Adaptive arc-length lookahead ───────────────────────────
    // Base lookahead grows with speed (time-gap) and shrinks on curved
    // sections so the guide does not cut corners on a tight detour.
    const double speed = state.velocity_world.norm();
    double lookahead = p_.macro_route_lookahead_min +
                       speed * p_.macro_guide_lookahead_time_gap_s;
    lookahead *= routeCurvatureFactor(w, seg_start, seg_len,
                                      progress + lookahead);
    lookahead = clamp(lookahead, p_.macro_route_lookahead_min,
                      p_.macro_route_lookahead_max);
    double guide_arc = progress + lookahead;
    if (force_advance) {
        // A no-progress refresh must move the carrot forward.  Merely
        // clearing hysteresis and recomputing the same deterministic guide
        // leaves a stationary vehicle with exactly the same target.
        double previous_arc = progress;
        double previous_lateral = 0.0;
        if (last_guide_valid_ &&
            projectOnRoute(w, seg_start, seg_len, last_guide_pos_,
                           previous_arc, previous_lateral)) {
            const double forced_step =
                std::max(0.5, 0.5 * p_.macro_guide_min_distance_m);
            guide_arc = std::max(guide_arc, previous_arc + forced_step);
        } else {
            guide_arc = std::max(
                guide_arc, progress + p_.macro_route_lookahead_max);
        }
        guide_arc = std::min(
            guide_arc,
            std::min(total, progress + p_.macro_route_lookahead_max));
    }
    lookahead_used = std::max(0.0, guide_arc - progress);
    Vec2d guide = pointAtArc(w, seg_start, seg_len, guide_arc, total);
    double guide_dist = (guide - state.position).norm();
    update_reason = force_advance ? "no_progress_advance"
                                  : "normal_advance";

    // ── 4) Hard minimum guide distance (never a foot-level target) ──
    if (guide_dist < p_.macro_guide_min_distance_m) {
        // The route ahead of the projection is too short to reach the
        // minimum distance: advance toward the route end / goal direction
        // so the 30 Hz planner always gets a directional target.
        bool advanced = false;
        for (int guard = 0; guard < 64; ++guard) {
            guide_arc += 0.5 * p_.macro_route_lookahead_min;
            if (guide_arc >= total) {
                guide_arc = total;
                break;
            }
            const Vec2d cand =
                pointAtArc(w, seg_start, seg_len, guide_arc, total);
            guide_dist = (cand - state.position).norm();
            if (guide_dist >= p_.macro_guide_min_distance_m) {
                guide = cand;
                advanced = true;
                break;
            }
        }
        if (!advanced) {
            // Even the route end is closer than the minimum distance: the
            // goal is the natural directional fallback.
            t.position = goal;
            update_reason = "fallback_goal_near_route_end";
            last_guide_valid_ = false;
            return t;
        }
        update_reason = force_advance ? "no_progress_min_distance_advance"
                                      : "min_distance_advance";
    }

    // ── 5) FOV clamp (guide usable by the 30 Hz expert) ────────────
    {
        const double fov_half = deg2rad(p_.obs_fov_deg) / 2.0;
        const double margin = deg2rad(p_.macro_guide_fov_margin_deg);
        auto bearingOf = [&](const Vec2d& p) {
            return wrapAngle(std::atan2(p.y() - state.position.y(),
                                        p.x() - state.position.x()) -
                             state.yaw);
        };
        if (std::fabs(bearingOf(guide)) > fov_half - margin) {
            // Pull the guide back along the route until it is inside the
            // FOV (but never below the minimum guide distance).
            const double step = std::max(0.1, 0.1 * lookahead);
            double a = guide_arc;
            bool found = false;
            while (a > progress) {
                a -= step;
                const Vec2d cand =
                    pointAtArc(w, seg_start, seg_len, a, total);
                if ((cand - state.position).norm() <
                    p_.macro_guide_min_distance_m) {
                    break;
                }
                if (std::fabs(bearingOf(cand)) <= fov_half - margin) {
                    guide = cand;
                    guide_arc = a;
                    found = true;
                    break;
                }
            }
            if (found) {
                update_reason = force_advance ? "no_progress_fov_clamp"
                                              : "fov_clamp";
            }
            // else: the whole min-distance prefix is outside the FOV — keep
            // the guide; the 30 Hz planner will TURN_TO_TARGET (tight route).
        }
    }

    // ── 6) Privileged chord-clearance clip (5 Hz only) ─────────────
    // The guide is ESDF-safe on the route; the straight chord from the
    // vehicle to the guide must also stay above the macro/local handoff
    // clearance (with the bounded recovery climb), so the 30 Hz expert is
    // never asked to cut a corner through the inflated blocker.
    {
        const double required_clearance = std::max(
            p_.lp_nominal_clearance_m,
            p_.scene_safety_clearance + p_.macro_route_clearance_margin +
                p_.lp_clearance_discretization_margin_m);
        const double chord_step = std::max(1e-3, 0.5 * esdf.resolution());
        auto chordClear = [&](const Vec2d& endpoint) {
            const Vec2d delta = endpoint - state.position;
            const double length = delta.norm();
            const int samples =
                std::max(1, static_cast<int>(std::ceil(length / chord_step)));
            const double start_clearance = esdf.sdfAt(state.position);
            bool recovered = start_clearance > required_clearance;
            double previous_clearance = start_clearance;
            for (int k = 1; k <= samples; ++k) {
                const double u = static_cast<double>(k) / samples;
                const double clearance =
                    esdf.sdfAt(state.position + u * delta);
                if (!recovered) {
                    // Bounded recovery: when the current pose is between
                    // scene safety and route clearance, the chord may climb
                    // out of that shell but may never move deeper.
                    if (clearance <= p_.scene_safety_clearance ||
                        clearance + 1e-6 < previous_clearance) {
                        return false;
                    }
                    recovered = clearance > required_clearance;
                } else if (clearance <= required_clearance) {
                    return false;
                }
                previous_clearance = clearance;
            }
            return true;
        };
        if (!chordClear(guide)) {
            // Pull back along the route to the farthest chord-clear point.
            const double step = std::max(0.1, 0.1 * lookahead);
            double a = guide_arc;
            bool found = false;
            while (a > progress) {
                a -= step;
                const Vec2d cand =
                    pointAtArc(w, seg_start, seg_len, a, total);
                if ((cand - state.position).norm() <
                    p_.macro_guide_min_distance_m) {
                    break;
                }
                if (chordClear(cand)) {
                    guide = cand;
                    guide_arc = a;
                    found = true;
                    break;
                }
            }
            if (found) {
                update_reason = force_advance ? "no_progress_chord_clip"
                                              : "chord_clip";
            } else {
                // Keep the directional route carrot.  The unified 30 Hz
                // planner still performs all observed-map safety checks;
                // the log now exposes that no long, straight privileged
                // chord exists instead of falsely claiming certification.
                update_reason = force_advance
                                    ? "no_progress_route_carrot"
                                    : "route_carrot_no_clear_chord";
            }
        }
    }

    // ── 7) World-space hysteresis / continuity ─────────────────────
    // A healthy new guide (>= min distance) that moved less than the
    // hysteresis threshold keeps the previous one, so a rolling route never
    // makes the 5 Hz target jitter left/right.  A foot-level old guide is
    // never held (the new healthy guide replaces it).
    if (!force_advance && last_guide_valid_ &&
        (guide - state.position).norm() >= p_.macro_guide_min_distance_m &&
        (guide - last_guide_pos_).norm() <= p_.macro_guide_hysteresis_m) {
        guide = last_guide_pos_;
        update_reason = "hysteresis_hold";
    }
    last_guide_pos_ = guide;
    last_guide_valid_ = true;
    lookahead_used = std::max(0.0, guide_arc - progress);

    t.position = guide;
    t.valid = true;
    t.is_macro_guide = true;
    t.update_event = 0;  // caller sets the event counter
    return t;
}

void MacroExpert5Hz::resetGuideState() {
    last_guide_pos_ = Vec2d(0.0, 0.0);
    last_guide_valid_ = false;
}

Vec2d MacroExpert5Hz::pointAtArc(const std::vector<Vec2d>& w,
                                 const std::vector<double>& seg_start,
                                 const std::vector<double>& seg_len,
                                 double arc, double total) const {
    const double a = clamp(arc, 0.0, total);
    for (size_t i = 0; i + 1 < w.size(); ++i) {
        if (a >= seg_start[i] && a <= seg_start[i + 1] + 1e-9) {
            const double frac =
                (a - seg_start[i]) / std::max(1e-9, seg_len[i]);
            return w[i] + (w[i + 1] - w[i]) * frac;
        }
    }
    return w.back();
}

bool MacroExpert5Hz::projectOnRoute(const std::vector<Vec2d>& w,
                                    const std::vector<double>& seg_start,
                                    const std::vector<double>& seg_len,
                                    const Vec2d& pos, double& progress,
                                    double& lateral) const {
    double best_sq = std::numeric_limits<double>::infinity();
    double best_prog = 0.0;
    double best_lat = 0.0;
    for (size_t i = 0; i + 1 < w.size(); ++i) {
        const Vec2d a = w[i], b = w[i + 1];
        const Vec2d ab = b - a;
        const double len2 = ab.squaredNorm();
        double tt = 0.0;
        if (len2 > 1e-12) {
            tt = clamp((pos - a).dot(ab) / len2, 0.0, 1.0);
        }
        const Vec2d proj = a + tt * ab;
        const double d2 = (proj - pos).squaredNorm();
        if (d2 < best_sq) {
            best_sq = d2;
            best_prog = seg_start[i] + tt * std::sqrt(len2);
            best_lat = std::sqrt(d2);
        }
    }
    if (!(best_sq < std::numeric_limits<double>::infinity())) return false;
    progress = best_prog;
    lateral = best_lat;
    return true;
}

double MacroExpert5Hz::routeCurvatureFactor(
    const std::vector<Vec2d>& w, const std::vector<double>& seg_start,
    const std::vector<double>& seg_len, double arc) const {
    const double window = 0.5 * p_.macro_route_lookahead_min;
    const double lo = std::max(0.0, arc - window);
    const double hi = std::min(seg_start.back(), arc + window);
    int n = 0;
    double turn_sum = 0.0;
    for (size_t i = 1; i + 1 < w.size(); ++i) {
        const double s = seg_start[i];
        if (s < lo || s > hi) continue;
        const Vec2d a = w[i] - w[i - 1];
        const Vec2d b = w[i + 1] - w[i];
        const double la = a.norm(), lb = b.norm();
        if (la < 1e-9 || lb < 1e-9) continue;
        turn_sum +=
            std::fabs(wrapAngle(std::atan2(cross2(a, b), a.dot(b))));
        ++n;
    }
    const double mean_turn = n > 0 ? turn_sum / n : 0.0;
    // 1.0 on straight sections, ~0.5 at high curvature.
    return clamp(1.0 - mean_turn / (M_PI / 3.0), 0.5, 1.0);
}

// ────────────────────────────────────────────────────────────────────
//  Exit-condition check
// ────────────────────────────────────────────────────────────────────
MacroExitCheck MacroExpert5Hz::checkExitConditions(
    const VehicleState2D& state, const Vec2d& goal,
    const PreviewResult& local_preview, bool blocker_passed) const {
    MacroExitCheck ec;

    // 1) Final goal re-entered the FOV with margin.
    const double beta =
        wrapAngle(std::atan2(goal.y() - state.position.y(),
                             goal.x() - state.position.x()) -
                  state.yaw);
    const double fov_half = deg2rad(p_.obs_fov_deg) / 2.0;
    const double margin = deg2rad(p_.macro_exit_fov_margin_deg);
    ec.goal_in_fov_margin = std::fabs(beta) <= fov_half - margin;

    // 2) The 30 Hz expert can independently generate a safe progressing
    //    trajectory toward the goal, or safely run final-goal terminal
    //    settling.  A generic SAFE_HOLD alone is not enough to leave macro
    //    mode — otherwise the macro would hand back control to a planner
    //    that immediately stalls.
    ec.local_has_progressing_trajectory =
        local_preview.success && !local_preview.turn_mode &&
        !local_preview.emergency_brake && local_preview.has_progressing_trajectory;
    // Near the final goal, positive progress intentionally vanishes while
    // the terminal controller settles translation/yaw.  That controller is
    // a valid handoff target as long as its trajectory is safe; the banned
    // case is a generic SAFE_HOLD/stop away from the goal.
    const bool terminal_settling_ready =
        local_preview.success && !local_preview.turn_mode &&
        !local_preview.emergency_brake &&
        local_preview.planner_status == PlannerStatus::TERMINAL_SETTLING;
    ec.local_precheck_ok =
        ec.local_has_progressing_trajectory || terminal_settling_ready;
    ec.local_executable_progress_m = local_preview.executable_progress_m;
    ec.local_safe_prefix_duration_s = local_preview.safe_prefix_duration_s;
    ec.local_output_speed_mps = local_preview.selected_output_speed_mps;

    // 3) The current blocker has been passed — provided by the FSM from
    //    the FIXED macro-entry route progress (goal-direction independent).
    ec.blocker_passed = blocker_passed;

    // 4) Not emergency braking / not recovering turn.
    ec.not_emergency_or_turn =
        !local_preview.emergency_brake && !local_preview.turn_mode;

    return ec;
}

}  // namespace il_2d_multiscale_debug
