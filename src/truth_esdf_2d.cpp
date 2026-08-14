#include "il_2d_multiscale_debug/truth_esdf_2d.hpp"

#include <cmath>
#include <limits>

namespace il_2d_multiscale_debug {

void TruthEsdf2D::build(const Scene2D& scene, double resolution) {
    scene_ = scene;
    resolution_ = resolution;
    width_ = static_cast<int>(std::ceil(
        (scene_.max_bounds.x() - scene_.min_bounds.x()) / resolution_));
    height_ = static_cast<int>(std::ceil(
        (scene_.max_bounds.y() - scene_.min_bounds.y()) / resolution_));
    grid_.assign(static_cast<size_t>(width_) * height_,
                 std::numeric_limits<double>::infinity());
    for (int iy = 0; iy < height_; ++iy) {
        for (int ix = 0; ix < width_; ++ix) {
            grid_[idx(ix, iy)] = sdfAt(cellCenter(ix, iy));
        }
    }
}

Vec2d TruthEsdf2D::cellCenter(int ix, int iy) const {
    return Vec2d(scene_.min_bounds.x() + (ix + 0.5) * resolution_,
                 scene_.min_bounds.y() + (iy + 0.5) * resolution_);
}

double TruthEsdf2D::sdfAt(const Vec2d& p) const {
    double d = std::numeric_limits<double>::infinity();
    // Distance to the inside of the rectangular boundary.
    const double bx = std::min(p.x() - scene_.min_bounds.x(),
                               scene_.max_bounds.x() - p.x());
    const double by = std::min(p.y() - scene_.min_bounds.y(),
                               scene_.max_bounds.y() - p.y());
    d = std::min(d, std::min(bx, by));
    // Distance to all cylinders.
    for (const auto& o : scene_.obstacles) {
        d = std::min(d, (p - o.center).norm() - o.radius);
    }
    return d;
}

int TruthEsdf2D::occupancyValueAt(int ix, int iy) const {
    if (!inGrid(ix, iy)) return 100;
    // Negative (inside obstacle / outside region) → 100; ≥ 0 → 0;
    // small positive values ramp up so the free space edge is visible.
    const double v = grid_[idx(ix, iy)];
    if (v < 0.0) return 100;
    const double scaled = clamp(v / (10.0 * resolution_), 0.0, 1.0);
    return static_cast<int>(std::lround((1.0 - scaled) * 100.0));
}

}  // namespace il_2d_multiscale_debug
