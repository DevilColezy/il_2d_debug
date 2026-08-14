#include "il_2d_multiscale_debug/scene_generator.hpp"

#include <cmath>

namespace il_2d_multiscale_debug {

int SceneGenerator::randInt(int lo, int hi) {
    std::uniform_int_distribution<int> d(lo, hi);
    return d(rng_);
}

double SceneGenerator::rand01() {
    std::uniform_real_distribution<double> d(0.0, 1.0);
    return d(rng_);
}

double SceneGenerator::sampleRadius() {
    const double rmin = p_.scene_min_radius, rmax = p_.scene_max_radius;
    const double u = rand01();
    if (p_.scene_radius_distribution == "log_uniform") {
        // Log-uniform covers small / medium / large scales evenly.
        const double lmin = std::log(std::max(1e-3, rmin));
        const double lmax = std::log(std::max(1e-3, rmax));
        return std::exp(lmin + u * (lmax - lmin));
    }
    return rmin + u * (rmax - rmin);
}

Vec2d SceneGenerator::sampleCenter(double r) {
    const double m = p_.scene_boundary_margin + p_.scene_safety_clearance;
    const double lo_x = p_.region_min_x + r + m;
    const double hi_x = p_.region_max_x - r - m;
    const double lo_y = p_.region_min_y + r + m;
    const double hi_y = p_.region_max_y - r - m;
    if (hi_x <= lo_x || hi_y <= lo_y) {
        // Obstacle too large for the region at this boundary margin —
        // return an invalid centre; rejection will resample the scene.
        return Vec2d(0.0, 0.0);
    }
    std::uniform_real_distribution<double> dx(lo_x, hi_x);
    std::uniform_real_distribution<double> dy(lo_y, hi_y);
    return Vec2d(dx(rng_), dy(rng_));
}

bool SceneGenerator::validPlacement(const Vec2d& c, double r,
                                    const std::vector<Obstacle2D>& existing) const {
    // Boundary clearance.
    const double bm = p_.scene_boundary_margin + p_.scene_safety_clearance;
    if (c.x() - r < p_.region_min_x + bm || c.x() + r > p_.region_max_x - bm ||
        c.y() - r < p_.region_min_y + bm || c.y() + r > p_.region_max_y - bm) {
        return false;
    }
    // Inter-obstacle surface gap.
    const double gap = 2.0 * p_.scene_safety_clearance + p_.scene_passage_margin;
    for (const auto& e : existing) {
        if ((c - e.center).norm() < r + e.radius + gap) return false;
    }
    return true;
}

bool SceneGenerator::placeOne(const std::vector<Obstacle2D>& existing,
                              Obstacle2D& out) {
    for (int a = 0; a < p_.scene_max_attempts_per_obstacle; ++a) {
        const double r = sampleRadius();
        const Vec2d c = sampleCenter(r);
        if (!validPlacement(c, r, existing)) continue;
        out = Obstacle2D{c, r, -1};
        return true;
    }
    return false;
}

SceneGenerationResult SceneGenerator::generate(uint64_t seed, uint64_t scene_id) {
    SceneGenerationResult res;
    res.requested_obstacle_count = -1;
    rng_.seed(seed);
    for (int sa = 0; sa < p_.scene_max_total_scene_attempts; ++sa) {
        const int count = randInt(p_.scene_min_obstacles, p_.scene_max_obstacles);
        res.requested_obstacle_count = count;
        if (count == 0) {
            // ONLY a random draw of zero obstacles may produce an empty
            // scene (zero-obstacle scenes are legal).
            Scene2D s;
            s.min_bounds = Vec2d(p_.region_min_x, p_.region_min_y);
            s.max_bounds = Vec2d(p_.region_max_x, p_.region_max_y);
            s.seed = seed;
            s.scene_id = scene_id;
            s.valid = true;
            res.scene = std::move(s);
            res.placed_obstacle_count = 0;
            res.success = true;
            res.reason = "ok (zero obstacles drawn)";
            return res;
        }
        std::vector<Obstacle2D> obs;
        obs.reserve(static_cast<size_t>(count));
        bool ok = true;
        for (int i = 0; i < count; ++i) {
            Obstacle2D o;
            if (!placeOne(obs, o)) {
                ok = false;  // resample the WHOLE scene
                break;
            }
            o.id = i;
            obs.push_back(o);
        }
        if (ok) {
            Scene2D s;
            s.min_bounds = Vec2d(p_.region_min_x, p_.region_min_y);
            s.max_bounds = Vec2d(p_.region_max_x, p_.region_max_y);
            s.obstacles = std::move(obs);
            s.seed = seed;
            s.scene_id = scene_id;
            s.valid = true;
            res.scene = std::move(s);
            res.placed_obstacle_count = count;
            res.success = true;
            res.reason = "ok";
            return res;
        }
    }
    // All whole-scene attempts exhausted for a NON-ZERO obstacle count:
    // report an explicit failure — never silently return an empty scene.
    res.success = false;
    res.placed_obstacle_count = 0;
    res.reason = "scene generation exhausted attempts for requested count " +
                 std::to_string(res.requested_obstacle_count) +
                 " (no legal placement found; try another seed)";
    return res;
}

}  // namespace il_2d_multiscale_debug
