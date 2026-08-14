#pragma once
/// @file   truth_esdf_2d.hpp
/// @brief  Global 2D signed ESDF built analytically from obstacle circles
///         and the rectangular region boundary.
///
///   esdf(p) = min( min over cylinders ( ||p-center|| - radius ),
///                  distance from p to the inside of the rectangle )
///
///   Negative inside obstacles / outside the region, positive in free
///   space.  The grid is sampled at cell centres with the configured
///   resolution.  This is PRIVILEGED TRUTH: it is used only by the
///   scene/task qualification, the human visualization, the collision
///   referee and the 5 Hz macro expert — never by the 30 Hz planner.

#include "il_2d_multiscale_debug/types.hpp"

#include <vector>

namespace il_2d_multiscale_debug {

class TruthEsdf2D {
public:
    TruthEsdf2D() = default;

    /// Build the ESDF grid for the given scene at the given resolution.
    void build(const Scene2D& scene, double resolution);

    /// Exact signed distance at an arbitrary world point (analytic).
    double sdfAt(const Vec2d& p) const;
    double sdfAt(double x, double y) const { return sdfAt(Vec2d(x, y)); }

    /// True iff sdfAt(p) > clearance  (selectable space is STRICTLY
    /// esdf > safety_clearance; never mixed with >=).
    bool isFree(const Vec2d& p, double clearance) const { return sdfAt(p) > clearance; }

    // Grid access ----------------------------------------------------
    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }
    const Scene2D& scene() const { return scene_; }
    const Vec2d& min_bounds() const { return scene_.min_bounds; }
    const Vec2d& max_bounds() const { return scene_.max_bounds; }
    double cellAt(int ix, int iy) const { return grid_[idx(ix, iy)]; }
    Vec2d cellCenter(int ix, int iy) const;
    bool cellFree(int ix, int iy, double clearance) const {
        return inGrid(ix, iy) && grid_[idx(ix, iy)] > clearance;
    }
    bool inGrid(int ix, int iy) const {
        return ix >= 0 && iy >= 0 && ix < width_ && iy < height_;
    }
    size_t idx(int ix, int iy) const { return static_cast<size_t>(iy) * width_ + ix; }

    /// World→grid index (clamped).
    int ixOf(double x) const {
        return static_cast<int>(std::floor((x - scene_.min_bounds.x()) / resolution_));
    }
    int iyOf(double y) const {
        return static_cast<int>(std::floor((y - scene_.min_bounds.y()) / resolution_));
    }

    /// Occupancy-grid style value in [0,100] for visualization
    /// (100 = most negative, 0 = at/above zero).  Latched truth topic.
    int occupancyValueAt(int ix, int iy) const;

private:
    Scene2D scene_;
    double resolution_ = 0.1;
    int width_ = 0, height_ = 0;
    std::vector<double> grid_;
};

}  // namespace il_2d_multiscale_debug
