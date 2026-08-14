#include "il_2d_multiscale_debug/connectivity_analyzer.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <queue>

namespace il_2d_multiscale_debug {

void ConnectivityAnalyzer::analyze(const TruthEsdf2D& esdf, double clearance,
                                   int neighbor) {
    width_ = esdf.width();
    height_ = esdf.height();
    labels_.assign(static_cast<size_t>(width_) * height_, -1);
    component_areas_.clear();
    main_component_id_ = -1;

    const int di[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dj[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    const int nconn = (neighbor == 4) ? 4 : 8;

    int label = 0;
    for (int iy = 0; iy < height_; ++iy) {
        for (int ix = 0; ix < width_; ++ix) {
            const size_t id = static_cast<size_t>(iy) * width_ + ix;
            if (labels_[id] != -1 || !esdf.cellFree(ix, iy, clearance)) continue;

            int area = 0;
            std::deque<std::pair<int, int>> stack;
            stack.emplace_back(ix, iy);
            labels_[id] = label;
            while (!stack.empty()) {
                const auto [cx, cy] = stack.front();
                stack.pop_front();
                ++area;
                for (int k = 0; k < nconn; ++k) {
                    const int nx = cx + dj[k];
                    const int ny = cy + di[k];
                    if (!esdf.inGrid(nx, ny) || !esdf.cellFree(nx, ny, clearance))
                        continue;
                    // No diagonal corner cutting: for a diagonal step both
                    // orthogonal neighbours must also be traversable.
                    if ((k % 2 == 1) &&
                        (!esdf.cellFree(cx + dj[k], cy, clearance) ||
                         !esdf.cellFree(cx, cy + di[k], clearance))) {
                        continue;
                    }
                    const size_t nid = static_cast<size_t>(ny) * width_ + nx;
                    if (labels_[nid] != -1) continue;
                    labels_[nid] = label;
                    stack.emplace_back(nx, ny);
                }
            }
            component_areas_.push_back(area);
            ++label;
        }
    }

    // Main component = largest area.
    main_component_id_ = -1;
    int best = -1;
    for (size_t c = 0; c < component_areas_.size(); ++c) {
        if (component_areas_[c] > best) {
            best = component_areas_[c];
            main_component_id_ = static_cast<int>(c);
        }
    }
}

bool ConnectivityAnalyzer::astarConnected(const TruthEsdf2D& esdf,
                                          double clearance, const Vec2d& a,
                                          const Vec2d& b) const {
    if (width_ == 0 || height_ == 0) return false;
    const int sx = esdf.ixOf(a.x()), sy = esdf.iyOf(a.y());
    const int gx = esdf.ixOf(b.x()), gy = esdf.iyOf(b.y());
    if (!esdf.cellFree(sx, sy, clearance) || !esdf.cellFree(gx, gy, clearance))
        return false;

    const int di[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    const int dj[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    const double w[8] = {1.0, M_SQRT2, 1.0, M_SQRT2, 1.0, M_SQRT2, 1.0, M_SQRT2};

    std::vector<double> gcost(static_cast<size_t>(width_) * height_,
                              std::numeric_limits<double>::infinity());
    std::vector<bool> closed(static_cast<size_t>(width_) * height_, false);

    auto h = [&](int ix, int iy) {
        const double dx = (ix - gx) * esdf.resolution();
        const double dy = (iy - gy) * esdf.resolution();
        return std::sqrt(dx * dx + dy * dy);
    };

    using Node = std::tuple<double, double, int, int>;  // f, g, ix, iy
    std::priority_queue<Node, std::vector<Node>,
                        std::greater<Node>>
        open;
    gcost[static_cast<size_t>(sy) * width_ + sx] = 0.0;
    open.emplace(h(sx, sy), 0.0, sx, sy);

    while (!open.empty()) {
        const auto [f, g, cx, cy] = open.top();
        open.pop();
        const size_t cid = static_cast<size_t>(cy) * width_ + cx;
        if (closed[cid]) continue;
        closed[cid] = true;
        if (cx == gx && cy == gy) return true;
        for (int k = 0; k < 8; ++k) {
            const int nx = cx + dj[k];
            const int ny = cy + di[k];
            if (!esdf.inGrid(nx, ny) || !esdf.cellFree(nx, ny, clearance)) continue;
            // No diagonal corner cutting (consistent with the flood fill).
            if ((k % 2 == 1) &&
                (!esdf.cellFree(cx + dj[k], cy, clearance) ||
                 !esdf.cellFree(cx, cy + di[k], clearance))) {
                continue;
            }
            const size_t nid = static_cast<size_t>(ny) * width_ + nx;
            if (closed[nid]) continue;
            const double ng = g + w[k] * esdf.resolution();
            if (ng < gcost[nid] - 1e-9) {
                gcost[nid] = ng;
                open.emplace(ng + h(nx, ny), ng, nx, ny);
            }
        }
    }
    return false;
}

}  // namespace il_2d_multiscale_debug
