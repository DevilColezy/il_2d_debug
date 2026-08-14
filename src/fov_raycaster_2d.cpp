#include "il_2d_multiscale_debug/fov_raycaster_2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace il_2d_multiscale_debug {

LocalObservation FovRaycaster2D::cast(const VehicleState2D& state,
                                      const Scene2D& scene,
                                      uint64_t tick) const {
    const double res = p_.obs_resolution;
    const double range = p_.obs_range_m;
    const double fov = deg2rad(p_.obs_fov_deg);
    const Vec2d& gmin = scene.min_bounds;

    // ── GRID-ALIGNED PATCH (root-cause fix) ─────────────────────────
    // The history map (ObservedGrid2D) is anchored at scene.min_bounds on
    // the global grid.  The instantaneous patch must use the SAME grid, so
    // its origin is snapped to a global grid boundary:
    //   global_ix_min = floor((vehicle_x - range - scene_min_x) / res)
    //   patch.origin.x = scene_min_x + global_ix_min * res
    // A patch cell centre then maps (via worldToGrid, floor semantics) to
    // EXACTLY the same global cell in the history map — the old
    // `state.position - Vec2d(range, range)` origin caused a one-cell
    // mis-registration (e.g. query (67,104) vs write (68,105)).
    // A tiny range epsilon guarantees full coverage of the vehicle±range
    // disc even when the max corner sits exactly on a grid boundary and
    // floating point rounds it just below it.
    const double cov_eps = 1e-6;
    const GridIndex2D lo = worldToGrid(state.position - Vec2d(range, range),
                                       gmin, res);
    const GridIndex2D hi = worldToGrid(
        state.position + Vec2d(range + cov_eps, range + cov_eps), gmin, res);

    LocalObservation patch;
    patch.resolution = res;
    patch.width = (hi.ix - lo.ix) + 1;
    patch.height = (hi.iy - lo.iy) + 1;
    patch.origin = Vec2d(gmin.x() + lo.ix * res, gmin.y() + lo.iy * res);
    patch.cells.assign(static_cast<size_t>(patch.width) * patch.height,
                       CellState::UNKNOWN);
    patch.age_ticks.assign(static_cast<size_t>(patch.width) * patch.height, 0);
    patch.max_age_ticks = 1;
    patch.tick = tick;

    auto patchCell = [&](double wx, double wy) {
        const GridIndex2D g = worldToGrid(Vec2d(wx, wy), patch.origin, res);
        if (!patch.inGrid(g.ix, g.iy)) return;
        patch.cells[patch.idx(g.ix, g.iy)] = CellState::FREE;
    };
    auto markOccupied = [&](double wx, double wy) {
        const GridIndex2D g = worldToGrid(Vec2d(wx, wy), patch.origin, res);
        if (!patch.inGrid(g.ix, g.iy)) return;
        patch.cells[patch.idx(g.ix, g.iy)] = CellState::OCCUPIED;
    };

    // ── Pass 1: angular ray march — marks FREE ahead and the first-hit
    //    OCCUPIED surface ring; everything beyond stays UNKNOWN. ──────
    // Ray bearings are  bearing(i) = -fov/2 + fov * i / n_rays  so the
    // FIRST and LAST rays lie EXACTLY on the FOV boundaries (no rounding
    // gaps at the edges).
    const double ray_da = deg2rad(p_.obs_ray_angular_res_deg);
    const int n_rays = std::max(
        1, static_cast<int>(std::ceil(fov / std::max(1e-9, ray_da))));
    const double march_step = res * 0.5;
    for (int i = 0; i <= n_rays; ++i) {
        const double bearing =
            -fov / 2.0 + fov * (static_cast<double>(i) / n_rays);
        const Vec2d dir(std::cos(state.yaw + bearing),
                        std::sin(state.yaw + bearing));
        bool hit = false;
        for (double d = 0.0; d <= range + 1e-9; d += march_step) {
            const Vec2d p = state.position + dir * d;
            // First surface contact?
            bool inside = false;
            for (const auto& o : scene.obstacles) {
                if ((p - o.center).norm() < o.radius) { inside = true; break; }
            }
            if (inside) {
                markOccupied(p.x(), p.y());
                hit = true;
                break;  // occluded behind stays UNKNOWN
            }
            patchCell(p.x(), p.y());
        }
        (void)hit;
    }

    // ── Pass 2: cell-wise classification for cells still UNKNOWN but
    //    inside the FOV and range — occlusion by any cylinder surface
    //    strictly before the cell keeps them UNKNOWN. ─────────────────
    for (int iy = 0; iy < patch.height; ++iy) {
        for (int ix = 0; ix < patch.width; ++ix) {
            const size_t id = patch.idx(ix, iy);
            if (patch.cells[id] != CellState::UNKNOWN) continue;
            const Vec2d cw = gridCellCenter(ix, iy, patch.origin, res);
            const Vec2d rel = cw - state.position;
            const double dist = rel.norm();
            if (dist > range + 0.5 * res) continue;  // beyond range → UNKNOWN
            if (dist < 1e-9) {
                // The vehicle's own cell: no ray to cast (avoids div-by-0),
                // the pose is in free space by construction → FREE.
                patch.cells[id] = CellState::FREE;
                continue;
            }
            const double bearing =
                wrapAngle(std::atan2(rel.y(), rel.x()) - state.yaw);
            if (std::fabs(bearing) > fov / 2.0 + 1e-9) continue;  // outside FOV

            const Vec2d dir = rel / dist;
            double first_hit = std::numeric_limits<double>::infinity();
            bool inside_any = false;
            for (const auto& o : scene.obstacles) {
                const Vec2d oc = o.center - state.position;
                const double proj = oc.dot(dir);
                const double perp2 = oc.squaredNorm() - proj * proj;
                if (perp2 > o.radius * o.radius) continue;
                const double along = proj - std::sqrt(std::max(0.0, o.radius * o.radius - perp2));
                if (along > 1e-6) first_hit = std::min(first_hit, along);
                if ((cw - o.center).norm() < o.radius) inside_any = true;
            }
            const bool occluded = first_hit < dist - 1e-3;
            if (inside_any && !occluded) {
                patch.cells[id] = CellState::OCCUPIED;  // visible surface cell
            } else if (!occluded) {
                patch.cells[id] = CellState::FREE;
            }
            // occluded → remains UNKNOWN
        }
    }

    // ── Sensor-origin cell is known usable ─────────────────────────
    // The sensor's own origin cell is always FREE after a valid cast: the
    // pose itself is usable space.  This models the sensor origin being
    // known, NOT truth leakage — the referee / planar simulator (truth)
    // still owns collision detection, so a vehicle inside an obstacle is
    // still caught there.  ONLY the single vehicle cell is touched (never
    // a neighbourhood), using the SAME worldToGrid conversion so it cannot
    // drift to a neighbouring cell.
    const GridIndex2D veh = worldToGrid(state.position, patch.origin, res);
    if (patch.inGrid(veh.ix, veh.iy)) {
        const size_t vid = patch.idx(veh.ix, veh.iy);
        patch.cells[vid] = CellState::FREE;
        patch.age_ticks[vid] = 0;
    }

    return patch;
}

}  // namespace il_2d_multiscale_debug
