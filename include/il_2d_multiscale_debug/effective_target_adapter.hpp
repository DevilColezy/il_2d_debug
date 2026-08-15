#pragma once
/// @file   effective_target_adapter.hpp
/// @brief  v9 EffectiveTargetAdapter — the single 30 Hz information
///         bottleneck between the 5 Hz VisibilityTargetCorrector and the
///         30 Hz expert.
///
/// Data flow (v9):
///   original_goal
///      ↓ (5 Hz, ZOH held)
///   TargetCorrectionDirective
///      (PASS_THROUGH / NORMAL_CORRECTION / TURN_LEFT / TURN_RIGHT)
///      ↓ (EVERY 30 Hz tick — EffectiveTargetAdapter::encode)
///   EncodedTargetInput
///      { direction_body (unit), normalized_distance,
///        effective_target_world }
///      ↓
///   LocalTarget world point  → current C++ LocalPlanner30Hz
///   direction_body + normalized_distance → future 30 Hz student labels
///
/// Encoding protocol (see README §3.5):
///   R        = observation/range_m
///   reserve  = target_encoding/normal_distance_reserve_m (0.5 m)
///   normal_distance = min(real_target_distance, R - reserve) / R
///     ⇒ ordinary targets are strictly < 1 (normal_distance_max ≈ 0.9167
///       for R=6, reserve=0.5);
///   TURN_LEFT / TURN_RIGHT ⇒ direction starts just outside the FOV and
///       is then latched in the world frame; normalized_distance == 1.0
///       EXACTLY and therefore commands pure rotation with no translation;
///   goal reached          ⇒ normalized_distance == 0, canonical
///       direction (1,0) so a near-zero distance never injects a random
///       direction into the labels.
///
/// Direction classification (student classes):
///   class 0          = TURN_LEFT
///   classes 1 .. N   = ordinary in-FOV bins ordered LEFT-to-RIGHT from
///                      +FOV/2-margin to -FOV/2+margin, MUST include the
///                      0° direction (N = direction_bin_count, odd);
///   class N + 1      = TURN_RIGHT.
/// The 5 Hz expert quantizes its continuous bearing through the SAME
/// quantizeBearing()/decodeDirectionToken() used by the student, so the
/// executed target is always reproducible from the student label.

#include "il_2d_multiscale_debug/types.hpp"

namespace il_2d_multiscale_debug {

class EffectiveTargetAdapter {
public:
    explicit EffectiveTargetAdapter(const Params2D& p) : p_(p) {}

    /// Encode the current (zero-order-held) directive at the live vehicle
    /// pose.  Called EVERY real 30 Hz tick.  For PASS_THROUGH /
    /// NORMAL_CORRECTION the effective world target is a real point. For
    /// TURN_* the direction is re-derived from a world-latched anchor, so
    /// its body bearing moves into the FOV instead of remaining outside
    /// forever. normalized_distance=1 keeps this a pure-rotation command.
    EncodedTargetInput encode(const VehicleState2D& state,
                              const Vec2d& original_goal,
                              const TargetCorrectionDirective& directive) const;

    // ── Student-interface helpers (shared with the 5 Hz expert so the
    //    expert and the student pass through the SAME quantization /
    //    decoding — the expert may never drive the 30 Hz layer with a
    //    continuous unquantized target and only log the quantized label).
    // ────────────────────────────────────────────────────────────────

    /// (R - reserve) in metres — maximum world distance of an ordinary
    /// encoded target.
    double normalMaxDistanceM() const {
        const double range = std::max(1e-9, p_.obs_range_m);
        // Keep a real numeric gap from the reserved TURN value even when
        // Params2D is constructed directly without params_io validation.
        const double reserve = std::max(
            std::max(1e-9, 1e-6 * range),
            p_.te_normal_distance_reserve_m);
        return std::max(0.0, range - reserve);
    }
    /// (R - reserve) / R — strict maximum of an ordinary normalized
    /// distance (< 1 by construction when reserve > 0).
    double normalDistanceMax() const {
        return normalMaxDistanceM() / std::max(1e-9, p_.obs_range_m);
    }
    /// Half span (rad) of the ordinary-bin coverage:
    ///   FOV/2 - target_encoding/turn_ray_margin_deg.
    double binHalfSpanRad() const {
        return deg2rad(p_.obs_fov_deg) / 2.0 -
               deg2rad(p_.te_turn_ray_margin_deg);
    }
    /// Ordinary bin width (rad) = 2 * half_span / (N - 1).
    double binWidthRad() const {
        const int n = std::max(3, p_.te_direction_bin_count);
        return 2.0 * binHalfSpanRad() / static_cast<double>(n - 1);
    }
    /// Continuous body bearing (rad, CCW + = LEFT) → student direction
    /// class: 0 = TURN_LEFT, 1..N = ordinary bins, N+1 = TURN_RIGHT.
    /// Deterministic nearest-bin quantization, tokens ordered from
    /// +half_span (LEFT) to -half_span (RIGHT).
    int quantizeBearing(double bearing_rad) const;
    /// Student direction class → ordinary bin CENTRE body bearing (rad).
    /// Valid only for ordinary classes (1..N); returns 0 otherwise.
    double tokenCenterBearingRad(int token) const;
    /// Student direction class → decoded body unit direction.
    Vec2d decodeDirectionToken(int token) const;
    /// Real world distance → clamped normalized distance:
    ///   min(d, R - reserve) / R ∈ [0, normal_distance_max].
    double clampNormalizedDistance(double dist) const;
    /// Normalized distance → world distance (dist = normalized * R).
    double normalizedToWorld(double normalized) const {
        return normalized * std::max(1e-9, p_.obs_range_m);
    }
    /// Initial body-frame bearing of a bounded TURN step (rad):
    /// ±(FOV/2 + margin).
    double turnBearingRad(SideSelection side) const;
    /// Initial body-frame unit direction of a bounded TURN step.
    Vec2d turnDirectionBody(SideSelection side) const;

private:
    Params2D p_;
};

}  // namespace il_2d_multiscale_debug
