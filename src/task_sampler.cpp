#include "il_2d_multiscale_debug/task_sampler.hpp"

#include <cmath>
#include <limits>

namespace il_2d_multiscale_debug {

namespace {
uint64_t mixSeed(uint64_t a, uint64_t b, uint64_t c) {
    // deterministic 64-bit mixing (splitmix-inspired)
    auto mix = [](uint64_t x) {
        x += 0x9e3779b97f4a7c15ULL;
        x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
        x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
        return x ^ (x >> 31);
    };
    return mix(a ^ mix(b ^ mix(c)));
}
}  // namespace

bool TaskSampler::randomFreeCellInMain(const Scene2D& scene,
                                       const TruthEsdf2D& esdf,
                                       const ConnectivityAnalyzer& conn,
                                       Vec2d& out) {
    // Collect selectable main-component cells.
    std::vector<std::pair<int, int>> cells;
    for (int iy = 0; iy < esdf.height(); ++iy) {
        for (int ix = 0; ix < esdf.width(); ++ix) {
            if (conn.labelAt(ix, iy) == conn.mainComponentId() &&
                esdf.cellFree(ix, iy, p_.scene_safety_clearance)) {
                cells.emplace_back(ix, iy);
            }
        }
    }
    if (cells.empty()) return false;
    std::uniform_int_distribution<size_t> d(0, cells.size() - 1);
    const auto [ix, iy] = cells[d(rng_)];
    out = esdf.cellCenter(ix, iy);
    return true;
}

bool TaskSampler::sampleTask(const Scene2D& scene, const TruthEsdf2D& esdf,
                             const ConnectivityAnalyzer& conn, uint64_t seed,
                             uint64_t task_id, Task2D& out,
                             std::string& reason) {
    rng_.seed(seed);
    if (conn.mainComponentAreaCells() == 0) {
        reason = "no main connected component";
        return false;
    }

    const int max_attempts = std::max(1, p_.task_max_sampling_attempts);
    int blocked_qual_attempts = 0;
    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        Vec2d start;
        if (!randomFreeCellInMain(scene, esdf, conn, start)) break;

        // Goal: random free cell in main component, far enough.
        for (int g = 0; g < max_attempts; ++g) {
            Vec2d goal;
            if (!randomFreeCellInMain(scene, esdf, conn, goal)) break;
            const double dist = (goal - start).norm();
            if (dist < p_.task_min_start_goal_distance) continue;
            if (!esdf.isFree(goal, p_.scene_safety_clearance)) continue;
            if (p_.task_astar_confirm &&
                !conn.astarConnected(esdf, p_.scene_safety_clearance, start, goal)) {
                continue;
            }

            // ── Causal qualification (route-level, both sides) ────────
            BlockerInfo dummy_blocker;
            if (findStraightBlocker(scene, start, goal, dummy_blocker)) {
                if (blocked_qual_attempts++ >= p_.task_max_blocked_qual_attempts) {
                    break;  // budget exhausted for expensive route checks
                }
                if (!causalQualify(scene, esdf, start, goal)) {
                    continue;  // rejected — keep sampling
                }
            }

            // Initial yaw: direction + random bias with random sign.
            const double dir = std::atan2(goal.y() - start.y(), goal.x() - start.x());
            std::uniform_real_distribution<double> bias_deg(
                p_.task_yaw_bias_min_deg, p_.task_yaw_bias_max_deg);
            const double mag = bias_deg(rng_);
            const double sign = (rng_() & 1) ? 1.0 : -1.0;
            const double yaw = wrapAngle(dir + sign * deg2rad(mag));

            out = Task2D{start, goal, yaw, task_id, scene.scene_id, seed, true};
            reason = "ok";
            return true;
        }
    }
    reason = "task sampling budget exhausted (no legal start/goal pair)";
    return false;
}

bool TaskSampler::findStraightBlocker(const Scene2D& scene, const Vec2d& start,
                                      const Vec2d& goal,
                                      BlockerInfo& blocker) const {
    const Vec2d axis = goal - start;
    const double axis_len = std::max(1e-6, axis.norm());
    // The straight corridor is blocked when an obstacle surface comes
    // within safety_clearance of the segment.
    const double margin =
        p_.scene_safety_clearance + p_.macro_route_clearance_margin +
        p_.lp_clearance_discretization_margin_m;
    bool blocked = false;
    double best_along = std::numeric_limits<double>::infinity();
    double best_pen = -std::numeric_limits<double>::infinity();
    for (const auto& o : scene.obstacles) {
        const double t = clamp(((o.center - start).dot(axis)) / (axis_len * axis_len),
                               0.0, 1.0);
        const Vec2d proj = start + axis * t;
        const double d = (o.center - proj).norm();
        const double pen = o.radius + margin - d;
        if (pen > 0.0) {
            blocked = true;
            const double along = t * axis_len;
            // Match runtime semantics: qualify the nearest forward
            // straight-corridor blocker, then break ties by penetration
            // and stable obstacle id.
            if (along < best_along - 1e-9 ||
                (std::fabs(along - best_along) <= 1e-9 &&
                 (pen > best_pen + 1e-9 ||
                  (std::fabs(pen - best_pen) <= 1e-9 &&
                   (!blocker.found || o.id < blocker.obstacle_ids.front()))))) {
                best_along = along;
                best_pen = pen;
                blocker.found = true;
                blocker.association = BlockerAssociation::MATCHED;
                blocker.center = o.center;
                blocker.radius = o.radius;
                blocker.cluster_id = o.id;
                blocker.obstacle_ids = {o.id};
            }
        }
    }
    return blocked;
}

bool TaskSampler::causalQualify(const Scene2D& scene, const TruthEsdf2D& esdf,
                                const Vec2d& start, const Vec2d& goal) const {
    BlockerInfo blocker;
    if (!findStraightBlocker(scene, start, goal, blocker)) return true;  // clear

    // Pre-generate the constrained LEFT / RIGHT routes around the blocker.
    LeftRightRoutePlanner rp(p_);
    const auto res = rp.planRoutes(scene, esdf, start, goal, blocker);

    if (p_.task_require_both_sides_feasible) {
        // Accept only if BOTH homotopy branches are truly feasible, so the
        // runtime visible-evidence decision can never pick an infeasible
        // side (which would otherwise be TASK_INVALID_FOR_CAUSAL_RULE).
        return res.left_valid && res.right_valid;
    }
    // Relaxed mode cannot reproduce the future trigger-time FOV evidence
    // here.  Require RIGHT, because ambiguity deterministically defaults
    // to RIGHT; accepting a LEFT-only task would knowingly admit a causal
    // runtime failure.
    return res.right_valid;
}

bool TaskSampler::snapToSelectable(const Scene2D& scene, const TruthEsdf2D& esdf,
                                   const ConnectivityAnalyzer& conn,
                                   const Vec2d& p, Vec2d& out) const {
    const int cx = esdf.ixOf(p.x());
    const int cy = esdf.iyOf(p.y());
    const int R = p_.task_goal_snap_max_radius_cells;
    for (int r = 0; r <= R; ++r) {
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                if (std::max(std::abs(dx), std::abs(dy)) != r) continue;
                const int ix = cx + dx, iy = cy + dy;
                if (!esdf.inGrid(ix, iy)) continue;
                if (conn.labelAt(ix, iy) == conn.mainComponentId() &&
                    esdf.cellFree(ix, iy, p_.scene_safety_clearance)) {
                    out = esdf.cellCenter(ix, iy);
                    return true;
                }
            }
        }
    }
    return false;
}

}  // namespace il_2d_multiscale_debug
