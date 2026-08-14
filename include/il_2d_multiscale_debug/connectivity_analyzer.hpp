#pragma once
/// @file   connectivity_analyzer.hpp
/// @brief  8-neighbourhood connected-component analysis of the ESDF
///         selectable space (esdf > safety_clearance) plus a global A*
///         static connectivity confirmation.
///
/// Used ONLY for scene/task qualification and (implicitly) as the base of
/// the privileged 5 Hz macro route search.  Never handed to the 30 Hz
/// local planner.

#include "il_2d_multiscale_debug/truth_esdf_2d.hpp"

#include <vector>

namespace il_2d_multiscale_debug {

class ConnectivityAnalyzer {
public:
    ConnectivityAnalyzer() = default;

    /// Recompute connected components of cells with esdf >= clearance.
    void analyze(const TruthEsdf2D& esdf, double clearance, int neighbor = 8);

    int mainComponentId() const { return main_component_id_; }
    size_t mainComponentAreaCells() const {
        return main_component_id_ >= 0
                   ? component_areas_[static_cast<size_t>(main_component_id_)]
                   : 0;
    }
    const std::vector<int>& componentAreas() const { return component_areas_; }

    bool inGrid(int ix, int iy) const {
        return ix >= 0 && iy >= 0 && ix < width_ && iy < height_;
    }
    /// Component label of a grid cell (-1 = not selectable).
    int labelAt(int ix, int iy) const {
        return inGrid(ix, iy) ? labels_[static_cast<size_t>(iy) * width_ + ix] : -1;
    }
    /// True iff the world point lies in the main component.
    bool isInMainComponent(const TruthEsdf2D& esdf, const Vec2d& p) const {
        int ix = esdf.ixOf(p.x());
        int iy = esdf.iyOf(p.y());
        return labelAt(ix, iy) == main_component_id_;
    }

    /// A* static connectivity confirmation between two world points
    /// through cells with esdf >= clearance.  Deterministic.
    bool astarConnected(const TruthEsdf2D& esdf, double clearance,
                        const Vec2d& a, const Vec2d& b) const;

private:
    void floodFill(const TruthEsdf2D& esdf, double clearance, int neighbor);

    int width_ = 0, height_ = 0;
    std::vector<int> labels_;           // -1 = not selectable, else component id
    std::vector<int> component_areas_;  // indexed by component id
    int main_component_id_ = -1;
};

}  // namespace il_2d_multiscale_debug
